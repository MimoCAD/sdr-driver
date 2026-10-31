//! A USRP B2xx in sdr-driver's shape, over sdr-uhd (channel 0).
//!
//! sdr-uhd is libuhd's own shape: a radio handle and separate streamers.
//! Here the radio is held by the stream it becomes, as the other drivers'
//! are, and two things the USRP alone can say are put to use:
//!
//! - **Loss, exactly.** Every received buffer carries the device time of
//!   its first sample. From the first one on, a buffer's place in the
//!   radio's sample stream is `(time − first) × rate`; where it lies past
//!   the samples handed out plus those already counted lost, the
//!   difference was lost (an overflow), and is counted.
//! - **Breaks, by the device clock.** A change made while receiving reads
//!   the device time just before and just after the request; each is a
//!   place in the stream, and they bracket the change. (How long the
//!   AD9364's synthesizer takes to settle after it is unmeasured.)

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use sdr_core::gain::{Stage, StageReading, nearest_level};
use sdr_core::{Backend, DeviceDesc, Error, Iq, RecvInfo, Result, TuneRequest, TuneResult};
use sdr_uhd::{Usrp, UsrpStream, UsrpTx};

use crate::device::{Caps, StreamConfig, WireFormat};
use crate::stream::{Break, Loss};

/// The B2xx family as UHD drives it: the AD9364's 50 MHz–6 GHz and
/// 220 kS/s–56 MS/s, 12-bit converters, a DDC after the LO.
pub(crate) const CAPS: Caps = Caps {
    backend: Backend::Usrp,
    freq_range: (50e6, 6e9),
    rate_range: (220e3, 56e6),
    transmit: true,
    native_bits: 12,
    wire: WireFormat::Sc16,
    has_dsp_shift: true,
};

/// The master clock a rate is given, as a multiple of the rate: decimation
/// 2 or more. Measured on a B206mini (an 11-configuration sweep): with the
/// clock equal to the rate (decimation 1) the AD9364 put symmetric sideband
/// clusters ±270–310 kHz around every strong carrier, −18 to −40 dBc, at
/// 19.2 and 24 Msps; with decimation 2 or more none appeared at any rate,
/// tuning or band tried.
const MASTER_CLOCK_PER_RATE: f64 = 2.0;
/// The AD9364's highest master clock, Hz (so above 30.72 Msps the clock
/// cannot be doubled).
const MAX_MASTER_CLOCK_HZ: f64 = 61.44e6;

/// The B2xx receive gain range UHD exposes for the AD9364: 0–76 dB in 1 dB
/// steps, one stage ([`Stage::Pga`]).
const RX_GAIN_MAX_DB: u8 = 76;
/// How long one `read` waits per round before trying again, seconds.
const READ_ROUND_S: f64 = 1.0;
/// At the finish, how long the drain waits for one more buffer, seconds.
const DRAIN_WAIT_S: f64 = 0.1;
/// UHD's per-send wait while its buffers are full, seconds.
const SEND_TIMEOUT_S: f64 = 1.0;

/// The receive gain stages UHD exposes for a B2xx: the one PGA.
pub(crate) const STAGES: [Stage; 1] = [Stage::Pga];

/// The PGA's levels, 0–76 dB.
fn pga_levels_db() -> Vec<f64> {
    (0..=RX_GAIN_MAX_DB).map(f64::from).collect()
}

/// A stage's levels; `None` for any but the PGA.
pub(crate) fn stage_levels_db(stage: Stage) -> Option<Vec<f64>> {
    (stage == Stage::Pga).then(pga_levels_db)
}

/// A stage this radio has, or [`Error::Unsupported`].
fn own(stage: Stage) -> Result<Stage> {
    if STAGES.contains(&stage) {
        Ok(stage)
    } else {
        Err(stage.absent(sdr_uhd::BACKEND))
    }
}

fn reading(db: f64) -> StageReading {
    let levels = usize::from(RX_GAIN_MAX_DB) + 1;
    StageReading {
        stage: Stage::Pga,
        level: nearest_level(&pga_levels_db(), db).min(levels - 1),
        levels,
        db,
    }
}

/// A field of a find result (`key=value,…`).
fn field<'a>(args: &'a str, key: &str) -> Option<&'a str> {
    args.split(',')
        .find_map(|kv| kv.trim().strip_prefix(key)?.strip_prefix('='))
}

/// Every attached USRP, as `uhd:<the args UHD found it by>`.
pub(crate) fn list() -> Vec<DeviceDesc> {
    sdr_uhd::find()
        .into_iter()
        .map(|args| DeviceDesc {
            label: format!(
                "{} (serial {})",
                field(&args, "product").unwrap_or("USRP"),
                field(&args, "serial").unwrap_or("?")
            ),
            spec: format!("uhd:{args}"),
            backend: Backend::Usrp,
        })
        .collect()
}

/// An open USRP: sdr-uhd's radio and its label.
pub struct Uhd {
    pub(crate) usrp: Usrp,
    label: String,
}

impl Uhd {
    /// The label from the motherboard's identity as UHD reports it.
    pub(crate) fn open(args: &str) -> Result<Uhd> {
        let usrp = Usrp::open(args)?;
        let info = usrp.mboard_info(0).unwrap_or_default();
        let or = |s: String, d: &str| if s.is_empty() { d.to_string() } else { s };
        Ok(Uhd {
            label: format!("{} (serial {})", or(info.id, "USRP"), or(info.serial, "?")),
            usrp,
        })
    }

    pub(crate) fn label(&self) -> &str {
        &self.label
    }

    /// The receive rate, with the master clock pinned at twice it
    /// ([`MASTER_CLOCK_PER_RATE`]): left to UHD, a B2xx can be given a clock
    /// equal to the rate (decimation 1), and the AD9364 then puts symmetric
    /// sideband clusters ±270–310 kHz around every strong carrier, −18 to
    /// −40 dBc. Above [`MAX_MASTER_CLOCK_HZ`] / 2 the clock cannot be
    /// doubled and decimation 1 is the only plan.
    pub(crate) fn set_rate(&mut self, sps: f64) -> Result<()> {
        self.usrp
            .set_master_clock_rate((MASTER_CLOCK_PER_RATE * sps).min(MAX_MASTER_CLOCK_HZ))?;
        self.usrp.set_rx_rate(sps, 0)
    }

    pub(crate) fn stage_readings(&self) -> Result<Vec<StageReading>> {
        Ok(vec![reading(self.usrp.rx_gain(0)?)])
    }

    pub(crate) fn set_stage_level(&mut self, stage: Stage, level: usize) -> Result<()> {
        own(stage)?;
        self.usrp
            .set_rx_gain(level.min(usize::from(RX_GAIN_MAX_DB)) as f64, 0)
    }
}

/// What a finished USRP stream did.
#[derive(Clone, Debug, PartialEq)]
pub struct UhdReport {
    /// Samples handed out, the tail included.
    pub samples: u64,
    /// Samples the timestamps showed missing.
    pub lost_samples: u64,
    /// The gaps they came in (one per overflow).
    pub gaps: u32,
    /// Buffers that came without a timestamp: a gap before one would not be
    /// seen.
    pub untimed_buffers: u32,
    /// Overflows the radio flagged with no timestamped buffer after them to
    /// measure the gap (one at the very end of a stream): samples were lost,
    /// how many is not known.
    pub unmeasured_overflows: u32,
    pub breaks: Vec<Break>,
}

impl UhdReport {
    pub(crate) fn lost(&self) -> Loss {
        match (
            self.lost_samples,
            self.untimed_buffers + self.unmeasured_overflows,
        ) {
            (0, 0) => Loss::None,
            (n, 0) => Loss::Exact(n),
            (n, _) => Loss::AtLeast(n.max(1)),
        }
    }
}

/// The ledger the timestamps keep, and what every handle reports.
struct State {
    /// The device time of the first sample, seconds.
    t0: Option<f64>,
    handed: u64,
    lost: u64,
    gaps: u32,
    untimed: u32,
    /// An overflow was flagged and no timestamped buffer has come since.
    overflow_pending: bool,
    breaks: Vec<Break>,
    freq: f64,
    gain: f64,
    finished: bool,
}

struct Shared {
    usrp: Mutex<Usrp>,
    label: String,
    rate: f64,
    max_num_samps: usize,
    state: Mutex<State>,
}

fn guard<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|p| p.into_inner())
}

impl Shared {
    /// A device time as a place among the samples handed out.
    fn place(&self, s: &State, t: f64) -> u64 {
        match s.t0 {
            Some(t0) => (((t - t0) * self.rate).round().max(0.0) as u64).saturating_sub(s.lost),
            None => 0,
        }
    }

    /// One buffer into the ledger. An overflow is flagged on a receive of its
    /// own; the gap it left shows in the next buffer's timestamp.
    fn account(&self, info: &RecvInfo) {
        let mut s = guard(&self.state);
        if info.overflow {
            s.overflow_pending = true;
        }
        if info.n_samples == 0 {
            return;
        }
        if info.time_spec.is_some() {
            s.overflow_pending = false;
        }
        match info.time_spec {
            Some(ts) => {
                let t = ts.full_secs as f64 + ts.frac_secs;
                let t0 = *s.t0.get_or_insert(t);
                let at = ((t - t0) * self.rate).round().max(0.0) as u64;
                let expected = s.handed + s.lost;
                if at > expected {
                    s.lost += at - expected;
                    s.gaps += 1;
                }
            }
            None => s.untimed += 1,
        }
        s.handed += info.n_samples as u64;
    }

    /// Run `f` on the radio, bracketed by the device clock.
    fn change<T>(&self, f: impl FnOnce(&mut Usrp) -> Result<T>) -> Result<(T, Break)> {
        let mut u = guard(&self.usrp);
        let before = u.time_now()?;
        let t = f(&mut u)?;
        let after = u.time_now()?;
        let (freq, gain) = (u.rx_freq(0)?, u.rx_gain(0)?);
        drop(u);
        let mut s = guard(&self.state);
        let at_sample = self.place(&s, before);
        let b = Break {
            at_sample,
            settled_at: self.place(&s, after).max(at_sample),
            rf_hz: freq,
            gap: core::time::Duration::ZERO,
            dropped: 0,
        };
        s.freq = freq;
        s.gain = gain;
        s.breaks.push(b);
        Ok((t, b))
    }
}

/// A USRP receiving ([`crate::Stream::Uhd`]).
pub struct UhdStream {
    read: UhdReadHalf,
    control: UhdControl,
}

/// The reading half of a USRP stream.
pub struct UhdReadHalf {
    shared: Arc<Shared>,
    rx: UsrpStream,
    started: bool,
    /// Native bytes received and not yet handed out by `read`.
    pending: Vec<u8>,
    scratch: Vec<Iq>,
}

/// The changing half of a USRP stream. Cheap to clone.
#[derive(Clone)]
pub struct UhdControl {
    shared: Arc<Shared>,
}

macro_rules! uhd_reports {
    ($t:ident, $($sh:tt)+) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                self.$($sh)+.rate
            }

            pub fn freq(&self) -> Result<f64> {
                Ok(guard(&self.$($sh)+.state).freq)
            }

            pub fn gain(&self) -> f64 {
                guard(&self.$($sh)+.state).gain
            }

            pub fn stage_readings(&self) -> Vec<StageReading> {
                vec![reading(self.gain())]
            }

            pub fn label(&self) -> String {
                self.$($sh)+.label.clone()
            }

            pub fn delivered(&self) -> u64 {
                guard(&self.$($sh)+.state).handed
            }

            pub fn breaks(&self) -> Vec<Break> {
                guard(&self.$($sh)+.state).breaks.clone()
            }

            pub fn max_num_samps(&self) -> usize {
                self.$($sh)+.max_num_samps
            }
        }
    };
}

uhd_reports!(UhdStream, control.shared);
uhd_reports!(UhdReadHalf, shared);
uhd_reports!(UhdControl, shared);

impl UhdControl {
    pub async fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
        self.shared.change(|u| u.set_rx_freq(req, 0))
    }

    pub async fn set_gain(&self, db: f64) -> Result<Break> {
        self.shared.change(|u| u.set_rx_gain(db, 0)).map(|(_, b)| b)
    }

    pub async fn set_stage_level(&self, stage: Stage, level: usize) -> Result<Break> {
        own(stage)?;
        self.set_gain(level.min(usize::from(RX_GAIN_MAX_DB)) as f64)
            .await
    }

    pub async fn set_bandwidth(&self, hz: f64) -> Result<Break> {
        self.shared
            .change(|u| u.set_rx_bandwidth(hz, 0))
            .map(|(_, b)| b)
    }

    /// The radio underneath (antennas, the device clock), locked: a change
    /// made through it is not placed.
    pub fn usrp(&self) -> MutexGuard<'_, Usrp> {
        guard(&self.shared.usrp)
    }
}

impl UhdReadHalf {
    pub async fn start(&mut self) -> Result<()> {
        if !self.started {
            self.rx.start_continuous()?;
            self.started = true;
        }
        Ok(())
    }

    pub async fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
        self.start().await?;
        let info = self.rx.recv_into(buf, timeout_secs)?;
        self.shared.account(&info);
        Ok(info)
    }

    /// Signed 16-bit little-endian I then Q.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        while self.pending.is_empty() {
            if guard(&self.shared.state).finished {
                return Ok(0);
            }
            let mut scratch = core::mem::take(&mut self.scratch);
            scratch.resize(self.rx.max_num_samps(), Iq::default());
            let info = self.recv_into(&mut scratch, READ_ROUND_S).await?;
            self.pending.extend(
                scratch[..info.n_samples]
                    .iter()
                    .flat_map(|s| [s.i.to_le_bytes(), s.q.to_le_bytes()])
                    .flatten(),
            );
            self.scratch = scratch;
        }
        let n = self.pending.len().min(buf.len());
        buf[..n].copy_from_slice(&self.pending[..n]);
        self.pending.drain(..n);
        Ok(n)
    }
}

/// The reading methods on the whole stream.
impl UhdStream {
    pub(crate) fn new(u: Uhd, cfg: &StreamConfig) -> core::result::Result<UhdStream, (Uhd, Error)> {
        let made = (|| {
            let rx = u.usrp.rx_streamer(0, &cfg.uhd_args, cfg.otw)?;
            Ok::<_, Error>((
                rx,
                u.usrp.rx_rate(0)?,
                u.usrp.rx_freq(0)?,
                u.usrp.rx_gain(0)?,
            ))
        })();
        let (rx, rate, freq, gain) = match made {
            Ok(m) => m,
            Err(e) => return Err((u, e)),
        };
        let state = State {
            t0: None,
            handed: 0,
            lost: 0,
            gaps: 0,
            untimed: 0,
            overflow_pending: false,
            breaks: Vec::new(),
            freq,
            gain,
            finished: false,
        };
        let shared = Arc::new(Shared {
            max_num_samps: rx.max_num_samps(),
            usrp: Mutex::new(u.usrp),
            label: u.label,
            rate,
            state: Mutex::new(state),
        });
        Ok(UhdStream {
            read: UhdReadHalf {
                shared: Arc::clone(&shared),
                rx,
                started: false,
                pending: Vec::new(),
                scratch: Vec::new(),
            },
            control: UhdControl { shared },
        })
    }

    pub fn split(self) -> (UhdReadHalf, UhdControl) {
        (self.read, self.control)
    }

    #[allow(clippy::result_large_err)]
    pub fn reunite(
        read: UhdReadHalf,
        control: UhdControl,
    ) -> core::result::Result<UhdStream, (UhdReadHalf, UhdControl)> {
        if !Arc::ptr_eq(&read.shared, &control.shared) || Arc::strong_count(&read.shared) != 2 {
            return Err((read, control));
        }
        Ok(UhdStream { read, control })
    }

    pub async fn start(&mut self) -> Result<()> {
        self.read.start().await
    }

    pub async fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
        self.read.recv_into(buf, timeout_secs).await
    }

    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.read.read(buf).await
    }

    pub async fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
        self.control.retune(req).await
    }

    pub async fn set_gain(&self, db: f64) -> Result<Break> {
        self.control.set_gain(db).await
    }

    pub async fn set_stage_level(&self, stage: Stage, level: usize) -> Result<Break> {
        self.control.set_stage_level(stage, level).await
    }

    pub async fn set_bandwidth(&self, hz: f64) -> Result<Break> {
        self.control.set_bandwidth(hz).await
    }

    /// The radio underneath, locked: a change made through it is not placed.
    pub fn usrp(&self) -> MutexGuard<'_, Usrp> {
        self.control.usrp()
    }

    /// Stop, take what is still on its way, and hand the radio back.
    pub(crate) async fn finish(self) -> Result<(Uhd, UhdReport, Vec<Iq>)> {
        let UhdStream { read, control } = self;
        let UhdReadHalf {
            shared,
            mut rx,
            started,
            pending,
            ..
        } = read;
        let mut tail = Vec::new();
        // Native bytes received and not handed out come back as samples.
        for p in pending.as_chunks::<4>().0 {
            tail.push(Iq {
                i: i16::from_le_bytes([p[0], p[1]]),
                q: i16::from_le_bytes([p[2], p[3]]),
            });
        }
        if started {
            rx.stop()?;
            let mut buf = vec![Iq::default(); rx.max_num_samps()];
            loop {
                let info = rx.recv_into(&mut buf, DRAIN_WAIT_S)?;
                // Accounted first: an overflow comes on an empty receive.
                shared.account(&info);
                if info.n_samples == 0 && !info.overflow {
                    break;
                }
                tail.extend_from_slice(&buf[..info.n_samples]);
            }
        }
        guard(&shared.state).finished = true;
        drop(rx);
        drop(shared);
        let shared = Arc::try_unwrap(control.shared).map_err(|_| Error::Device {
            backend: sdr_uhd::BACKEND,
            context: "finish",
            message: "a clone of the control still holds the radio".into(),
        })?;
        let s = shared.state.into_inner().unwrap_or_else(|p| p.into_inner());
        let report = UhdReport {
            samples: s.handed,
            lost_samples: s.lost,
            gaps: s.gaps,
            untimed_buffers: s.untimed,
            unmeasured_overflows: s.overflow_pending as u32,
            breaks: s.breaks,
        };
        let usrp = shared.usrp.into_inner().unwrap_or_else(|p| p.into_inner());
        Ok((
            Uhd {
                usrp,
                label: shared.label,
            },
            report,
            tail,
        ))
    }
}

struct TxShared {
    usrp: Mutex<Usrp>,
    label: String,
    rate: f64,
    written: AtomicU64,
}

/// A USRP transmitting ([`crate::TxStream::Uhd`]).
pub struct UhdTxStream {
    write: UhdWriteHalf,
    control: UhdTxControl,
}

/// The writing half of a USRP transmission.
pub struct UhdWriteHalf {
    shared: Arc<TxShared>,
    tx: UsrpTx,
}

/// The changing half of a USRP transmission. Cheap to clone.
#[derive(Clone)]
pub struct UhdTxControl {
    shared: Arc<TxShared>,
}

macro_rules! uhd_tx_reports {
    ($t:ident, $($sh:tt)+) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                self.$($sh)+.rate
            }

            pub fn written(&self) -> u64 {
                self.$($sh)+.written.load(Ordering::Relaxed)
            }

            pub fn label(&self) -> String {
                self.$($sh)+.label.clone()
            }
        }
    };
}

uhd_tx_reports!(UhdTxStream, control.shared);
uhd_tx_reports!(UhdWriteHalf, shared);
uhd_tx_reports!(UhdTxControl, shared);

impl UhdWriteHalf {
    pub async fn send(&mut self, iq: &[Iq]) -> Result<()> {
        self.tx.send(iq, SEND_TIMEOUT_S)?;
        self.shared
            .written
            .fetch_add(iq.len() as u64, Ordering::Relaxed);
        Ok(())
    }

    pub fn samples_per_transfer(&self) -> usize {
        self.tx.max_num_samps()
    }
}

impl UhdTxControl {
    pub async fn retune(&self, req: &TuneRequest) -> Result<TuneResult> {
        guard(&self.shared.usrp).set_tx_freq(req, 0)
    }

    /// The radio underneath (transmit gain, antenna), locked.
    pub fn usrp(&self) -> MutexGuard<'_, Usrp> {
        guard(&self.shared.usrp)
    }
}

impl UhdTxStream {
    pub(crate) fn new(u: Uhd) -> core::result::Result<UhdTxStream, (Uhd, Error)> {
        let made = (|| Ok::<_, Error>((u.usrp.tx_streamer(0)?, u.usrp.tx_rate(0)?)))();
        let (tx, rate) = match made {
            Ok(m) => m,
            Err(e) => return Err((u, e)),
        };
        let shared = Arc::new(TxShared {
            usrp: Mutex::new(u.usrp),
            label: u.label,
            rate,
            written: AtomicU64::new(0),
        });
        Ok(UhdTxStream {
            write: UhdWriteHalf {
                shared: Arc::clone(&shared),
                tx,
            },
            control: UhdTxControl { shared },
        })
    }

    pub fn split(self) -> (UhdWriteHalf, UhdTxControl) {
        (self.write, self.control)
    }

    pub fn reunite(
        write: UhdWriteHalf,
        control: UhdTxControl,
    ) -> core::result::Result<UhdTxStream, (UhdWriteHalf, UhdTxControl)> {
        if !Arc::ptr_eq(&write.shared, &control.shared) || Arc::strong_count(&write.shared) != 2 {
            return Err((write, control));
        }
        Ok(UhdTxStream { write, control })
    }

    pub async fn send(&mut self, iq: &[Iq]) -> Result<()> {
        self.write.send(iq).await
    }

    pub fn samples_per_transfer(&self) -> usize {
        self.write.samples_per_transfer()
    }

    pub async fn retune(&self, req: &TuneRequest) -> Result<TuneResult> {
        self.control.retune(req).await
    }

    /// End the burst and hand the radio back, with the samples written.
    pub(crate) async fn finish(self) -> Result<(Uhd, u64)> {
        let UhdTxStream { write, control } = self;
        let UhdWriteHalf { shared, mut tx } = write;
        tx.end_burst()?;
        drop(tx);
        drop(shared);
        let shared = Arc::try_unwrap(control.shared).map_err(|_| Error::Device {
            backend: sdr_uhd::BACKEND,
            context: "finish",
            message: "a clone of the control still holds the radio".into(),
        })?;
        let written = shared.written.load(Ordering::Relaxed);
        let usrp = shared.usrp.into_inner().unwrap_or_else(|p| p.into_inner());
        Ok((
            Uhd {
                usrp,
                label: shared.label,
            },
            written,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn find_fields_read_by_key() {
        let args = "type=b200,name=B206i,serial=0000001,product=B206mini";
        assert_eq!(field(args, "product"), Some("B206mini"));
        assert_eq!(field(args, "serial"), Some("0000001"));
        assert_eq!(field(args, "mboard"), None);
    }

    #[test]
    fn the_pga_has_77_levels() {
        assert_eq!(stage_levels_db(Stage::Pga).map(|l| l.len()), Some(77));
        assert_eq!(stage_levels_db(Stage::Lna), None);
    }
}
