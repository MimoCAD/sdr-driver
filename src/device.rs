//! The radio: one enum over the compiled-in drivers, opened by spec.

use sdr_core::gain::{Stage, StageReading};
use sdr_core::{Backend, DeviceDesc, Error, OtwFormat, Result, TuneRequest, TuneResult};

use crate::stream::Stream;
use crate::tx::TxStream;

/// How a radio puts samples on the wire — what [`Stream::read_native`]
/// hands out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireFormat {
    /// Unsigned 8-bit I then Q, offset 128 (RTL2832U).
    OffsetU8,
    /// Signed 8-bit I then Q (HackRF).
    I8,
    /// Signed 16-bit little-endian I then Q (USRP; the host format
    /// whatever its wire format).
    Sc16,
}

/// What a radio can do, known before anything is asked of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Caps {
    pub backend: Backend,
    /// The tunable RF range, Hz.
    pub freq_range: (f64, f64),
    /// The sample rates it delivers, samples per second.
    pub rate_range: (f64, f64),
    /// It can transmit ([`Device::into_tx_stream`]).
    pub transmit: bool,
    /// Bits per I or Q at its converter.
    pub native_bits: u8,
    /// What [`Stream::read_native`] hands out.
    pub wire: WireFormat,
    /// A digital tuning stage after the RF LO (a USRP's DDC); without one a
    /// manual RF offset leaves the residual shift to the caller's mixer.
    pub has_dsp_shift: bool,
}

/// How a stream is made. Fields that do not apply to a radio are ignored.
#[derive(Clone, Debug, Default)]
pub struct StreamConfig {
    /// USB radios: bytes per bulk transfer (0: the driver's default).
    pub transfer_bytes: usize,
    /// USB radios: transfers kept in flight (0: the driver's default).
    pub queue_depth: usize,
    /// USRP: the wire format (USB bandwidth against precision).
    pub otw: OtwFormat,
    /// USRP: UHD's extra stream arguments, e.g. `num_recv_frames=1024`.
    pub uhd_args: String,
}

/// A stream could not be made; the radio is handed back.
pub struct StreamError {
    pub device: Box<Device>,
    pub error: Error,
}

impl core::fmt::Debug for StreamError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("StreamError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

impl core::fmt::Display for StreamError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

impl core::error::Error for StreamError {}

/// An open radio of any compiled-in driver.
pub enum Device {
    #[cfg(feature = "rtl")]
    Rtl(sdr_rtl::Rtl),
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::HackRf),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::Uhd),
}

impl core::fmt::Debug for Device {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("Device").field(&self.label()).finish()
    }
}

/// Every attached radio of the compiled-in drivers, without opening any,
/// best first: USRP (widest rate, 12 bits), HackRF, RTL-SDR. In a browser,
/// those the page has been granted.
pub async fn list() -> Vec<DeviceDesc> {
    #[allow(unused_mut)]
    let mut out = Vec::new();
    #[cfg(feature = "uhd")]
    out.extend(crate::uhd::list());
    #[cfg(feature = "hackrf")]
    out.extend(sdr_hackrf::list().await);
    #[cfg(feature = "rtl")]
    out.extend(sdr_rtl::list().await);
    out
}

impl Device {
    /// Open a radio by spec:
    ///
    /// - `rtl:`, `rtl:<index>`, `rtl:serial=<sn>` — an RTL-SDR
    /// - `hackrf:`, `hackrf:serial=<sn>` — a HackRF
    /// - `uhd:<args>` — a USRP, UHD's args verbatim; `b200:<args>` adds
    ///   `type=b200` when absent
    /// - `""` — the first radio found, in [`list`]'s order
    pub async fn open(spec: &str) -> Result<Device> {
        match parse_spec(spec)? {
            Spec::First => Self::open_first().await,
            Spec::Rtl(rest) => Self::open_rtl(rest).await,
            Spec::HackRf(rest) => Self::open_hackrf(rest).await,
            Spec::Uhd(args) => Self::open_uhd(&args),
        }
    }

    /// The first radio that opens, USRP then HackRF then RTL-SDR. When none
    /// does, the error says why each backend failed — a permissions problem
    /// is not "no radio".
    async fn open_first() -> Result<Device> {
        #[allow(unused_mut)]
        let mut why: Vec<String> = Vec::new();
        #[cfg(feature = "uhd")]
        match Self::open_uhd("") {
            Ok(d) => return Ok(d),
            Err(e) => why.push(format!("uhd: {e}")),
        }
        #[cfg(feature = "hackrf")]
        match Self::open_hackrf("").await {
            Ok(d) => return Ok(d),
            Err(e) => why.push(format!("hackrf: {e}")),
        }
        #[cfg(feature = "rtl")]
        match Self::open_rtl("").await {
            Ok(d) => return Ok(d),
            Err(e) => why.push(format!("rtl: {e}")),
        }
        Err(Error::NotFound {
            spec: format!("(any radio) — {}", why.join("; ")),
        })
    }

    #[allow(unused_variables)]
    async fn open_rtl(rest: &str) -> Result<Device> {
        #[cfg(feature = "rtl")]
        return Ok(Device::Rtl(sdr_rtl::Rtl::open(rest).await?));
        #[cfg(not(feature = "rtl"))]
        Err(crate::not_built("rtl"))
    }

    #[allow(unused_variables)]
    async fn open_hackrf(rest: &str) -> Result<Device> {
        #[cfg(feature = "hackrf")]
        return Ok(Device::HackRf(sdr_hackrf::HackRf::open(rest).await?));
        #[cfg(not(feature = "hackrf"))]
        Err(crate::not_built("hackrf"))
    }

    #[allow(unused_variables)]
    fn open_uhd(args: &str) -> Result<Device> {
        #[cfg(feature = "uhd")]
        return Ok(Device::Uhd(crate::uhd::Uhd::open(args)?));
        #[cfg(not(feature = "uhd"))]
        Err(crate::not_built("uhd"))
    }

    /// In a browser: ask the user for a radio (the WebUSB device chooser,
    /// filtered to every compiled-in USB driver's IDs; it must follow a
    /// user gesture, such as a click), then open it. The page keeps the
    /// grant, so later [`Device::open`] calls find it without asking.
    #[cfg(target_arch = "wasm32")]
    pub async fn request() -> Result<Device> {
        #[allow(unused_mut)]
        let mut selectors: Vec<nusb::DeviceSelector> = Vec::new();
        #[cfg(feature = "rtl")]
        selectors.extend(
            sdr_rtl::KNOWN_IDS
                .iter()
                .map(|&(v, p)| nusb::DeviceSelector::all().with_vid_pid(v, p)),
        );
        #[cfg(feature = "hackrf")]
        selectors.extend(
            sdr_hackrf::proto::KNOWN_IDS
                .iter()
                .map(|&(v, p, _)| nusb::DeviceSelector::all().with_vid_pid(v, p)),
        );
        let info = nusb::request_device(&selectors)
            .await
            .map_err(|e| Error::Usb {
                backend: "sdr-driver",
                context: "request device",
                message: e.to_string(),
            })?
            .ok_or_else(|| Error::NotFound {
                spec: "(no device chosen)".into(),
            })?;
        let by_serial = info
            .serial_number()
            .map(|s| format!("serial={s}"))
            .unwrap_or_default();
        let id = (info.vendor_id(), info.product_id());
        #[cfg(feature = "rtl")]
        if sdr_rtl::KNOWN_IDS.contains(&id) {
            return Self::open_rtl(&by_serial).await;
        }
        #[cfg(feature = "hackrf")]
        if sdr_hackrf::proto::KNOWN_IDS
            .iter()
            .any(|&(v, p, _)| (v, p) == id)
        {
            return Self::open_hackrf(&by_serial).await;
        }
        Err(Error::NotFound {
            spec: format!("{:04x}:{:04x} (no driver for it in this build)", id.0, id.1),
        })
    }

    /// What the radio can do.
    pub fn caps(&self) -> Caps {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(_) => Caps {
                backend: Backend::Rtl,
                freq_range: sdr_rtl::RF_RANGE_HZ,
                rate_range: sdr_rtl::demod::OUTPUT_RATE_GOOD_SPS,
                transmit: false,
                native_bits: 8,
                wire: WireFormat::OffsetU8,
                has_dsp_shift: false,
            },
            #[cfg(feature = "hackrf")]
            Device::HackRf(_) => Caps {
                backend: Backend::HackRf,
                freq_range: sdr_hackrf::FREQ_RANGE_HZ,
                rate_range: sdr_hackrf::RATE_RANGE_SPS,
                transmit: true,
                native_bits: 8,
                wire: WireFormat::I8,
                has_dsp_shift: false,
            },
            #[cfg(feature = "uhd")]
            Device::Uhd(_) => crate::uhd::CAPS,
        }
    }

    /// The radio's label (product and serial).
    pub fn label(&self) -> String {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.label().to_string(),
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h.label().to_string(),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.label().to_string(),
        }
    }

    pub async fn set_rate(&mut self, sps: f64) -> Result<()> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.set_rate(sps).await,
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h.set_rate(sps).await,
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.set_rate(sps),
        }
    }

    /// The rate as set (a USRP's as it settled).
    pub fn rate(&self) -> Result<f64> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => Ok(r.rate_set()),
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => Ok(h.rate()),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.usrp.rx_rate(0),
        }
    }

    pub async fn set_freq(&mut self, req: &TuneRequest) -> Result<TuneResult> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.set_freq(req).await,
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h.set_freq(req).await,
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.usrp.set_rx_freq(req, 0),
        }
    }

    /// The RF the samples are centred on.
    pub fn freq(&self) -> Result<f64> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.freq(),
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => Ok(h.freq()),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.usrp.rx_freq(0),
        }
    }

    /// Overall receive gain, dB, spread over the radio's stages.
    pub async fn set_gain(&mut self, db: f64) -> Result<()> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.set_gain(db).await,
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h.set_gain(db).await,
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.usrp.set_rx_gain(db, 0),
        }
    }

    pub fn gain(&self) -> Result<f64> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => Ok(r.gain()),
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => Ok(h.gain()),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.usrp.rx_gain(0),
        }
    }

    /// The gain stages this radio has, in signal order.
    pub fn stages(&self) -> Vec<Stage> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(_) => sdr_rtl::Stage::ALL.map(Stage::from).to_vec(),
            #[cfg(feature = "hackrf")]
            Device::HackRf(_) => sdr_hackrf::Stage::ALL.map(Stage::from).to_vec(),
            #[cfg(feature = "uhd")]
            Device::Uhd(_) => crate::uhd::STAGES.to_vec(),
        }
    }

    /// A stage's settable levels in dB, lowest first; `None` for a stage
    /// this radio does not have (see [`Device::stages`]).
    pub fn stage_levels_db(&self, stage: Stage) -> Option<Vec<f64>> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => sdr_rtl::Stage::try_from(stage)
                .ok()
                .map(|s| r.stage_levels_db(s)),
            #[cfg(feature = "hackrf")]
            Device::HackRf(_) => sdr_hackrf::Stage::try_from(stage)
                .ok()
                .map(sdr_hackrf::gain::stage_levels_db),
            #[cfg(feature = "uhd")]
            Device::Uhd(_) => crate::uhd::stage_levels_db(stage),
        }
    }

    /// Where each stage sits (as read back where the radio can say, as
    /// last set where it cannot).
    pub fn stage_readings(&self) -> Result<Vec<StageReading>> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => Ok(r
                .stage_readings()
                .into_iter()
                .map(StageReading::common)
                .collect()),
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => Ok(h
                .stage_readings()
                .into_iter()
                .map(StageReading::common)
                .collect()),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.stage_readings(),
        }
    }

    /// One stage to a level of [`Device::stage_levels_db`]; an
    /// [`Error::Unsupported`] for a stage this radio does not have.
    pub async fn set_stage_level(&mut self, stage: Stage, level: usize) -> Result<()> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.set_stage_level(stage.try_into()?, level).await,
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h.set_stage_level(stage.try_into()?, level).await,
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.set_stage_level(stage, level),
        }
    }

    /// The analog filter's bandwidth, Hz.
    pub async fn set_bandwidth(&mut self, hz: f64) -> Result<()> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r.set_bandwidth(hz).await,
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h.set_bandwidth(hz).await,
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => u.usrp.set_rx_bandwidth(hz, 0),
        }
    }

    /// The radio receiving, as a [`Stream`] at the rate it holds; it owns
    /// the radio until [`Stream::finish`] hands it back.
    pub async fn into_stream(
        self,
        cfg: &StreamConfig,
    ) -> core::result::Result<Stream, StreamError> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => r
                .into_stream(cfg.transfer_bytes, cfg.queue_depth)
                .await
                .map(Stream::Rtl)
                .map_err(|e| StreamError {
                    device: Box::new(Device::Rtl(*e.rtl)),
                    error: e.error,
                }),
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h
                .into_stream(cfg.transfer_bytes, cfg.queue_depth)
                .await
                .map(Stream::HackRf)
                .map_err(|e| StreamError {
                    device: Box::new(Device::HackRf(*e.hackrf)),
                    error: e.error,
                }),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => {
                crate::uhd::UhdStream::new(u, cfg)
                    .map(Stream::Uhd)
                    .map_err(|(u, error)| StreamError {
                        device: Box::new(Device::Uhd(u)),
                        error,
                    })
            }
        }
    }

    /// The radio transmitting, as a [`TxStream`] at the rate it holds
    /// (transmit rate and frequency: [`TxStream::retune`] or the driver's
    /// own settings first). A radio that cannot transmit
    /// ([`Caps::transmit`]) is handed back.
    #[cfg_attr(not(feature = "hackrf"), allow(unused_variables))]
    pub async fn into_tx_stream(
        self,
        cfg: &StreamConfig,
    ) -> core::result::Result<TxStream, StreamError> {
        match self {
            #[cfg(feature = "rtl")]
            Device::Rtl(r) => {
                let error = Error::Unsupported {
                    backend: sdr_rtl::BACKEND,
                    what: "transmit (the RTL2832U only receives)".into(),
                };
                Err(StreamError {
                    device: Box::new(Device::Rtl(r)),
                    error,
                })
            }
            #[cfg(feature = "hackrf")]
            Device::HackRf(h) => h
                .into_tx_stream(cfg.transfer_bytes, cfg.queue_depth)
                .await
                .map(TxStream::HackRf)
                .map_err(|e| StreamError {
                    device: Box::new(Device::HackRf(*e.hackrf)),
                    error: e.error,
                }),
            #[cfg(feature = "uhd")]
            Device::Uhd(u) => {
                crate::uhd::UhdTxStream::new(u)
                    .map(TxStream::Uhd)
                    .map_err(|(u, error)| StreamError {
                        device: Box::new(Device::Uhd(u)),
                        error,
                    })
            }
        }
    }

    /// The RTL-SDR underneath, if it is one (its EEPROM, the tuner's
    /// registers, its AGC).
    #[cfg(feature = "rtl")]
    pub fn as_rtl(&mut self) -> Option<&mut sdr_rtl::Rtl> {
        match self {
            Device::Rtl(r) => Some(r),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }

    /// The HackRF underneath, if it is one (the amp, the bias tee, the M0
    /// counters, transmit gain).
    #[cfg(feature = "hackrf")]
    pub fn as_hackrf(&mut self) -> Option<&mut sdr_hackrf::HackRf> {
        match self {
            Device::HackRf(h) => Some(h),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }

    /// The USRP underneath, if it is one (antennas, the device clock,
    /// transmit settings).
    #[cfg(feature = "uhd")]
    pub fn as_uhd(&mut self) -> Option<&mut sdr_uhd::Usrp> {
        match self {
            Device::Uhd(u) => Some(&mut u.usrp),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }
}

/// A device spec, parsed ([`Device::open`]'s forms).
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Spec<'a> {
    /// `""`: the first radio found.
    First,
    /// `rtl:<rest>`.
    Rtl(&'a str),
    /// `hackrf:<rest>`.
    HackRf(&'a str),
    /// `uhd:<args>` verbatim, or `b200:<args>` with `type=b200` added
    /// when absent.
    Uhd(String),
}

pub(crate) fn parse_spec(spec: &str) -> Result<Spec<'_>> {
    if spec.is_empty() {
        return Ok(Spec::First);
    }
    let (scheme, rest) = spec.split_once(':').unwrap_or((spec, ""));
    match scheme {
        "rtl" => Ok(Spec::Rtl(rest)),
        "hackrf" => Ok(Spec::HackRf(rest)),
        "uhd" => Ok(Spec::Uhd(rest.to_string())),
        "b200" => Ok(Spec::Uhd(
            if rest
                .split(',')
                .any(|kv| kv.trim_start().starts_with("type="))
            {
                rest.to_string()
            } else if rest.is_empty() {
                "type=b200".to_string()
            } else {
                format!("type=b200,{rest}")
            },
        )),
        _ => Err(Error::NotFound {
            spec: format!("{spec} (schemes: rtl:, hackrf:, uhd:, b200:)"),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn specs_parse_to_their_backend() {
        assert_eq!(parse_spec("").unwrap(), Spec::First);
        assert_eq!(parse_spec("rtl:").unwrap(), Spec::Rtl(""));
        assert_eq!(parse_spec("rtl").unwrap(), Spec::Rtl(""));
        assert_eq!(parse_spec("rtl:1").unwrap(), Spec::Rtl("1"));
        assert_eq!(
            parse_spec("rtl:serial=00000001").unwrap(),
            Spec::Rtl("serial=00000001")
        );
        assert_eq!(
            parse_spec("hackrf:serial=abc").unwrap(),
            Spec::HackRf("serial=abc")
        );
        assert_eq!(
            parse_spec("uhd:serial=1,master_clock_rate=1e6").unwrap(),
            Spec::Uhd("serial=1,master_clock_rate=1e6".into())
        );
    }

    /// `b200:` adds `type=b200` only when no `type=` key is present — a
    /// value merely containing "type=" does not count.
    #[test]
    fn b200_adds_its_type_once() {
        assert_eq!(parse_spec("b200:").unwrap(), Spec::Uhd("type=b200".into()));
        assert_eq!(
            parse_spec("b200:serial=1").unwrap(),
            Spec::Uhd("type=b200,serial=1".into())
        );
        assert_eq!(
            parse_spec("b200:type=b200,serial=1").unwrap(),
            Spec::Uhd("type=b200,serial=1".into())
        );
        assert_eq!(
            parse_spec("b200:name=subtype=x").unwrap(),
            Spec::Uhd("type=b200,name=subtype=x".into())
        );
    }

    #[test]
    fn an_unknown_scheme_names_the_known_ones() {
        let e = parse_spec("airspy:").unwrap_err().to_string();
        assert!(e.contains("rtl:") && e.contains("b200:"), "{e}");
    }
}
