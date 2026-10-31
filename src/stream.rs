//! The radio receiving: one enum over each driver's stream, its halves,
//! and what a finished stream reports.

use core::time::Duration;

use sdr_core::gain::{Stage, StageReading};
use sdr_core::{Iq, RecvInfo, Result, TuneRequest, TuneResult};

use crate::device::{Device, WireFormat};

/// Each variant's `$s` bound to the driver's own value, `$body` the same
/// text for all of them (the drivers' streams share their method names).
macro_rules! on {
    ($e:expr, $ty:ident, $s:ident => $body:expr) => {
        match $e {
            #[cfg(feature = "rtl")]
            $ty::Rtl($s) => $body,
            #[cfg(feature = "hackrf")]
            $ty::HackRf($s) => $body,
            #[cfg(feature = "uhd")]
            $ty::Uhd($s) => $body,
        }
    };
}

/// One change made while samples flowed. On every radio: every sample
/// before `at_sample` (from 0, counting what was handed out) is the old
/// configuration, every one from `settled_at` the new.
///
/// The radios differ in what lies between, and `gap` says which:
/// - **RTL-SDR:** the change is made inside a pause. The radio stops
///   sampling: sample `at_sample - 1` and sample `at_sample` are `gap`
///   apart in time, not one sample period — and a little more, since the
///   endpoint FIFO's last ~0.5-0.9 ms of the old configuration is
///   discarded at the break by design (sdr-rtl findings 16.6). A consumer
///   keeping time across the break adds `gap`. `at_sample` and
///   `settled_at` are 40 samples apart (the demodulator's pipeline: the
///   new configuration measured 5-32 samples after `at_sample`).
/// - **HackRF, USRP:** samples keep flowing (`gap` is zero) and
///   `at_sample`/`settled_at` bracket the request, read from the radio's
///   own count (the HackRF's M0 byte count, the USRP's clock).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Break {
    pub at_sample: u64,
    pub settled_at: u64,
    /// The RF the samples after the break are centred on, Hz.
    pub rf_hz: f64,
    /// How long the radio was not sampling at this break, at least: the
    /// RTL-SDR's pause (host clock); zero where samples keep flowing.
    pub gap: Duration,
    /// Samples the radio dropped at this break beyond the planned gap, as
    /// it counts them (the RTL-SDR's counter, read while paused); 0 where
    /// the radio has no such count.
    pub dropped: u64,
}

#[cfg(feature = "rtl")]
impl From<sdr_rtl::Break> for Break {
    fn from(b: sdr_rtl::Break) -> Break {
        let dropped =
            b.dropped_before.blocks as u64 * (sdr_rtl::protocol::BLK_DROP_BLOCK_BYTES as u64 / 2);
        Break {
            at_sample: b.at_sample,
            settled_at: b.settled_at,
            rf_hz: b.rf_hz,
            gap: b.quiet,
            dropped,
        }
    }
}

#[cfg(feature = "hackrf")]
impl From<sdr_hackrf::Break> for Break {
    fn from(b: sdr_hackrf::Break) -> Break {
        Break {
            at_sample: b.at_sample,
            settled_at: b.settled_at,
            rf_hz: b.rf_hz,
            gap: Duration::ZERO,
            dropped: 0,
        }
    }
}

/// Samples a stream lost without meaning to, as the radio counts them. A
/// change's planned gap is not loss: it is in [`Break::gap`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Loss {
    /// None lost.
    None,
    /// Exactly this many, as the radio counts (an RTL-SDR's dropped-block
    /// counter, which counts every block its FIFO overflowed — sdr-rtl
    /// findings 16 found it disagreeing with measured shortfalls only when
    /// control requests were made while samples flowed, which this driver
    /// does not do; a USRP's timestamps).
    Exact(u64),
    /// At least this many: the count is a floor (a HackRF keeps only its
    /// longest shortfall; an RTL-SDR's counter stopped at its maximum; a
    /// USRP overflow without a timestamp after it).
    AtLeast(u64),
}

/// An RTL-SDR stream's [`Loss`] from its two counts: the chip's counter
/// (blocks of 94 samples) and the host's ledger. Exact from the counter
/// while it is below 255 and the host saw no more than it; otherwise a
/// floor at the larger.
#[cfg(feature = "rtl")]
pub(crate) fn rtl_loss(dropped_blocks: u64, saturated: bool, host_lost: u64) -> Loss {
    let chip = dropped_blocks * (sdr_rtl::protocol::BLK_DROP_BLOCK_BYTES as u64 / 2);
    if saturated || host_lost > chip {
        Loss::AtLeast(chip.max(host_lost))
    } else if chip == 0 {
        Loss::None
    } else {
        Loss::Exact(chip)
    }
}

/// A HackRF stream's [`Loss`]: the firmware counts shortfalls and keeps the
/// longest, so the total is at least that.
#[cfg(feature = "hackrf")]
pub(crate) fn hackrf_loss(shortfalls: u64, longest: u64) -> Loss {
    if shortfalls == 0 {
        Loss::None
    } else {
        Loss::AtLeast(longest)
    }
}

#[cfg(test)]
mod loss_tests {
    #[allow(unused_imports)]
    use super::*;

    /// The counter's blocks are 94 samples; a counter stopped at 255 is
    /// only a floor.
    #[cfg(feature = "rtl")]
    #[test]
    fn rtl_loss_is_exact_only_below_saturation() {
        assert_eq!(rtl_loss(0, false, 0), Loss::None);
        assert_eq!(rtl_loss(3, false, 0), Loss::Exact(3 * 94));
        assert_eq!(
            rtl_loss(3, false, 100),
            Loss::Exact(3 * 94),
            "the host saw less: the chip's count stands"
        );
        assert_eq!(rtl_loss(255, true, 0), Loss::AtLeast(255 * 94));
        assert_eq!(rtl_loss(0, true, 0), Loss::AtLeast(0));
    }

    /// The host saw more than the chip counted: a floor at the host's.
    #[cfg(feature = "rtl")]
    #[test]
    fn rtl_loss_takes_the_host_count_when_larger() {
        assert_eq!(rtl_loss(0, false, 8_192), Loss::AtLeast(8_192));
        assert_eq!(rtl_loss(2, false, 8_192), Loss::AtLeast(8_192));
        assert_eq!(rtl_loss(255, true, 1_000_000), Loss::AtLeast(1_000_000));
    }

    /// One shortfall or many, the firmware keeps only the longest: a floor.
    #[cfg(feature = "hackrf")]
    #[test]
    fn hackrf_loss_is_a_floor() {
        assert_eq!(hackrf_loss(0, 0), Loss::None);
        assert_eq!(hackrf_loss(1, 500), Loss::AtLeast(500));
        assert_eq!(hackrf_loss(9, 500), Loss::AtLeast(500));
    }

    /// Review D6: the RTL path through this crate, end to end on sdr-rtl's
    /// simulated dongle — open, stream, retune, finish — with the samples
    /// in order and the break's gap and the report's loss as documented.
    #[cfg(all(feature = "rtl", not(target_arch = "wasm32")))]
    #[test]
    fn the_rtl_path_runs_end_to_end_on_the_simulated_dongle() {
        use sdr_core::{TuneRequest, block_on};
        use sdr_rtl::fake::{self, FakeRtl};
        block_on(async {
            let f = FakeRtl::new();
            let rtl = sdr_rtl::Rtl::open_transport(f.clone(), fake::label())
                .await
                .expect("open");
            let dev = Device::Rtl(rtl);
            assert_eq!(dev.caps().wire, WireFormat::OffsetU8);
            let mut s = dev
                .into_stream(&crate::StreamConfig::default())
                .await
                .map_err(|e| e.error)
                .expect("stream");
            let mut bytes = Vec::new();
            let mut buf = [0u8; 4_095];
            while bytes.len() < 200_000 {
                let n = s.read_native(&mut buf).await.expect("read");
                bytes.extend_from_slice(&buf[..n]);
            }
            let (_, b) = s.retune(&TuneRequest::auto(450e6)).await.expect("retune");
            assert!(b.gap > Duration::ZERO, "an RTL break has a gap");
            assert_eq!(b.dropped, 0);
            while bytes.len() < 400_000 {
                let n = s.read_native(&mut buf).await.expect("read");
                bytes.extend_from_slice(&buf[..n]);
            }
            let in_order = bytes
                .as_chunks::<2>()
                .0
                .iter()
                .enumerate()
                .all(|(k, p)| *p == fake::sample_bytes(k as u64));
            assert!(in_order, "samples in order across the break");
            let done = s.finish().await.expect("finish");
            assert_eq!(done.report.lost, Loss::None);
            assert_eq!(done.report.breaks.len(), 1);
            assert!(matches!(done.device, Device::Rtl(_)));
        });
    }

    /// An RTL-SDR break carries its gap and the blocks the pause cost.
    #[cfg(feature = "rtl")]
    #[test]
    fn an_rtl_break_carries_its_gap() {
        let b: Break = sdr_rtl::Break {
            at_sample: 10,
            settled_at: 42,
            dropped_before: sdr_rtl::DroppedBlocks {
                blocks: 2,
                saturated: false,
            },
            quiet: Duration::from_millis(61),
            rf_hz: 1e8,
        }
        .into();
        assert_eq!((b.gap, b.dropped), (Duration::from_millis(61), 2 * 94));
    }
}

/// What a finished stream did.
#[derive(Clone, Debug, PartialEq)]
pub struct StreamReport {
    /// Samples handed out, the tail included.
    pub samples: u64,
    pub lost: Loss,
    pub breaks: Vec<Break>,
    /// The driver's own report, with what only it counts.
    pub detail: BackendReport,
}

/// A driver's own stream report.
#[derive(Clone, Debug, PartialEq)]
pub enum BackendReport {
    #[cfg(feature = "rtl")]
    Rtl(sdr_rtl::StreamReport),
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::StreamReport),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdReport),
}

/// A finished stream: the radio back, the report, and the samples still on
/// their way when it finished.
pub struct Finished {
    pub device: Device,
    pub report: StreamReport,
    pub tail: Vec<Iq>,
}

/// The radio receiving. See the crate documentation.
pub enum Stream {
    #[cfg(feature = "rtl")]
    Rtl(sdr_rtl::Stream),
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::Stream),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdStream),
}

/// The reading half of a [`Stream`] ([`Stream::split`]).
pub enum ReadHalf {
    #[cfg(feature = "rtl")]
    Rtl(sdr_rtl::ReadHalf),
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::ReadHalf),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdReadHalf),
}

/// The changing half of a [`Stream`] ([`Stream::split`]). Cheap to clone.
#[derive(Clone)]
pub enum Control {
    #[cfg(feature = "rtl")]
    Rtl(sdr_rtl::Control),
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::Control),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdControl),
}

/// What every handle reports at once.
macro_rules! reports {
    ($t:ident) => {
        impl $t {
            /// The sample rate (fixed for the stream's life).
            pub fn rate(&self) -> f64 {
                on!(self, $t, s => s.rate())
            }

            /// The RF the samples are centred on, Hz.
            pub fn freq(&self) -> Result<f64> {
                match self {
                    #[cfg(feature = "rtl")]
                    $t::Rtl(s) => s.freq(),
                    #[cfg(feature = "hackrf")]
                    $t::HackRf(s) => Ok(s.freq()),
                    #[cfg(feature = "uhd")]
                    $t::Uhd(s) => s.freq(),
                }
            }

            /// The receive gain as last set, dB.
            pub fn gain(&self) -> f64 {
                on!(self, $t, s => s.gain())
            }

            /// Each gain stage as last set.
            pub fn stage_readings(&self) -> Vec<StageReading> {
                on!(self, $t, s => s.stage_readings().into_iter().map(StageReading::common).collect())
            }

            /// The radio's label.
            pub fn label(&self) -> String {
                on!(self, $t, s => s.label())
            }

            /// Samples handed out so far.
            pub fn delivered(&self) -> u64 {
                on!(self, $t, s => s.delivered())
            }

            /// The breaks made so far.
            pub fn breaks(&self) -> Vec<Break> {
                on!(self, $t, s => s.breaks().into_iter().map(Into::into).collect())
            }

            /// Samples one receive hands out at most.
            pub fn max_num_samps(&self) -> usize {
                on!(self, $t, s => s.max_num_samps())
            }
        }
    };
}

/// What changes the radio while it receives; each returns its [`Break`].
macro_rules! changes {
    ($t:ident) => {
        impl $t {
            /// Retune while receiving.
            pub async fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
                on!(self, $t, s => s.retune(req).await.map(|(t, b)| (t, b.into())))
            }

            /// The overall receive gain while receiving, dB.
            pub async fn set_gain(&self, db: f64) -> Result<Break> {
                on!(self, $t, s => s.set_gain(db).await.map(Into::into))
            }

            /// One gain stage while receiving; an [`Error::Unsupported`]
            /// for a stage this radio does not have.
            ///
            /// [`Error::Unsupported`]: sdr_core::Error::Unsupported
            pub async fn set_stage_level(&self, stage: Stage, level: usize) -> Result<Break> {
                match self {
                    #[cfg(feature = "rtl")]
                    $t::Rtl(s) => s.set_stage_level(stage.try_into()?, level).await.map(Into::into),
                    #[cfg(feature = "hackrf")]
                    $t::HackRf(s) => s.set_stage_level(stage.try_into()?, level).await.map(Into::into),
                    #[cfg(feature = "uhd")]
                    $t::Uhd(s) => s.set_stage_level(stage, level).await,
                }
            }

            /// The analog filter's bandwidth while receiving, Hz.
            pub async fn set_bandwidth(&self, hz: f64) -> Result<Break> {
                on!(self, $t, s => s.set_bandwidth(hz).await.map(Into::into))
            }
        }
    };
}

/// What reads the samples.
macro_rules! reads {
    ($t:ident) => {
        impl $t {
            /// Samples into `buf`, as [`Iq`] (full scale ±32768); at most
            /// `timeout_secs` waiting (`timed_out` then set). Starts the
            /// stream on the first call.
            pub async fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
                on!(self, $t, s => s.recv_into(buf, timeout_secs).await)
            }

            /// The radio's own wire bytes into `buf` ([`WireFormat`]), no
            /// conversion: waits for at least one byte; 0 once the stream
            /// has finished. Starts the stream on the first call.
            pub async fn read_native(&mut self, buf: &mut [u8]) -> Result<usize> {
                on!(self, $t, s => s.read(buf).await)
            }

            /// What [`Self::read_native`] hands out.
            pub fn wire_format(&self) -> WireFormat {
                match self {
                    #[cfg(feature = "rtl")]
                    $t::Rtl(_) => WireFormat::OffsetU8,
                    #[cfg(feature = "hackrf")]
                    $t::HackRf(_) => WireFormat::I8,
                    #[cfg(feature = "uhd")]
                    $t::Uhd(_) => WireFormat::Sc16,
                }
            }

            /// Start the stream now rather than at the first read.
            pub async fn start(&mut self) -> Result<()> {
                on!(self, $t, s => s.start().await)
            }
        }
    };
}

reports!(Stream);
reports!(ReadHalf);
reports!(Control);
changes!(Stream);
changes!(Control);
reads!(Stream);
reads!(ReadHalf);

impl Stream {
    /// The two halves, each usable on its own thread or task.
    pub fn split(self) -> (ReadHalf, Control) {
        match self {
            #[cfg(feature = "rtl")]
            Stream::Rtl(s) => {
                let (r, c) = s.split();
                (ReadHalf::Rtl(r), Control::Rtl(c))
            }
            #[cfg(feature = "hackrf")]
            Stream::HackRf(s) => {
                let (r, c) = s.split();
                (ReadHalf::HackRf(r), Control::HackRf(c))
            }
            #[cfg(feature = "uhd")]
            Stream::Uhd(s) => {
                let (r, c) = s.split();
                (ReadHalf::Uhd(r), Control::Uhd(c))
            }
        }
    }

    /// The halves of one stream joined again; handed back if they are not
    /// the same stream's, or a clone of the control still exists.
    // The halves come back whole when they do not match; reuniting is
    // rare, so their size there costs nothing.
    #[allow(clippy::result_large_err)]
    pub fn reunite(
        read: ReadHalf,
        control: Control,
    ) -> core::result::Result<Stream, (ReadHalf, Control)> {
        match (read, control) {
            #[cfg(feature = "rtl")]
            (ReadHalf::Rtl(r), Control::Rtl(c)) => sdr_rtl::Stream::reunite(r, c)
                .map(Stream::Rtl)
                .map_err(|(r, c)| (ReadHalf::Rtl(r), Control::Rtl(c))),
            #[cfg(feature = "hackrf")]
            (ReadHalf::HackRf(r), Control::HackRf(c)) => sdr_hackrf::Stream::reunite(r, c)
                .map(Stream::HackRf)
                .map_err(|(r, c)| (ReadHalf::HackRf(r), Control::HackRf(c))),
            #[cfg(feature = "uhd")]
            (ReadHalf::Uhd(r), Control::Uhd(c)) => crate::uhd::UhdStream::reunite(r, c)
                .map(Stream::Uhd)
                .map_err(|(r, c)| (ReadHalf::Uhd(r), Control::Uhd(c))),
            #[allow(unreachable_patterns)]
            other => Err(other),
        }
    }

    /// Stop with nothing lost unseen, hand out the samples still on their
    /// way, and read the radio's own loss count: the radio back, the
    /// report, the tail.
    pub async fn finish(self) -> Result<Finished> {
        match self {
            #[cfg(feature = "rtl")]
            Stream::Rtl(s) => {
                let f = s.finish().await?;
                let tail = f.tail_samples();
                let r = &f.report;
                let lost = rtl_loss(r.dropped_blocks, r.saturated, r.host_lost_samples);
                let report = StreamReport {
                    samples: r.samples,
                    lost,
                    breaks: r.breaks.iter().copied().map(Into::into).collect(),
                    detail: BackendReport::Rtl(f.report.clone()),
                };
                Ok(Finished {
                    device: Device::Rtl(f.rtl),
                    report,
                    tail,
                })
            }
            #[cfg(feature = "hackrf")]
            Stream::HackRf(s) => {
                let f = s.finish().await?;
                let tail = f.tail_samples();
                let r = &f.report;
                let lost = hackrf_loss(r.shortfalls as u64, r.longest_shortfall);
                let report = StreamReport {
                    samples: r.samples,
                    lost,
                    breaks: r.breaks.iter().copied().map(Into::into).collect(),
                    detail: BackendReport::HackRf(f.report.clone()),
                };
                Ok(Finished {
                    device: Device::HackRf(f.hackrf),
                    report,
                    tail,
                })
            }
            #[cfg(feature = "uhd")]
            Stream::Uhd(s) => {
                let (u, r, tail) = s.finish().await?;
                let report = StreamReport {
                    samples: r.samples,
                    lost: r.lost(),
                    breaks: r.breaks.clone(),
                    detail: BackendReport::Uhd(r),
                };
                Ok(Finished {
                    device: Device::Uhd(u),
                    report,
                    tail,
                })
            }
        }
    }

    /// The RTL-SDR's own stream, if it is one.
    #[cfg(feature = "rtl")]
    pub fn as_rtl(&mut self) -> Option<&mut sdr_rtl::Stream> {
        match self {
            Stream::Rtl(s) => Some(s),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }

    /// The HackRF's own stream, if it is one.
    #[cfg(feature = "hackrf")]
    pub fn as_hackrf(&mut self) -> Option<&mut sdr_hackrf::Stream> {
        match self {
            Stream::HackRf(s) => Some(s),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }

    /// The USRP's stream, if it is one.
    #[cfg(feature = "uhd")]
    pub fn as_uhd(&mut self) -> Option<&mut crate::uhd::UhdStream> {
        match self {
            Stream::Uhd(s) => Some(s),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }
}
