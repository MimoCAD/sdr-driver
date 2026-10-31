//! The same API, blocking: each method runs its async counterpart to
//! completion on the calling thread (`sdr_core::block_on`). Native only — a
//! browser cannot block. The receive streams are [`std::io::Read`] (the
//! radio's own wire bytes, [`crate::WireFormat`]) and `sdr_core::RxSource`.
//!
//! ```no_run
//! use sdr_driver::blocking::Device;
//! use sdr_driver::{Iq, StreamConfig, TuneRequest};
//!
//! let mut radio = Device::open("")?; // the first radio found
//! radio.set_rate(2.4e6)?;
//! radio.set_freq(&TuneRequest::auto(100.1e6))?;
//! let (mut reader, control) = radio.into_stream(&StreamConfig::default()).map_err(|e| e.error)?.split();
//! let mut buf = vec![Iq::default(); reader.max_num_samps()];
//! let info = reader.recv_into(&mut buf, 1.0)?;
//! let (_, brk) = control.retune(&TuneRequest::auto(100.3e6))?; // brk.settled_at: the new tuning from here
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

use std::io;

use sdr_core::gain::{Stage, StageReading};
use sdr_core::{
    DeviceDesc, Error, Iq, RecvInfo, Result, RxSource, TuneRequest, TuneResult, block_on,
    elevate_current_thread,
};

use crate::{Break, Caps, StreamConfig, StreamReport, TxReport, WireFormat};

/// Every attached radio ([`crate::list`]).
pub fn list() -> Vec<DeviceDesc> {
    block_on(crate::list())
}

/// An open radio ([`crate::Device`]), blocking.
#[derive(Debug)]
pub struct Device(crate::Device);

impl From<crate::Device> for Device {
    fn from(d: crate::Device) -> Device {
        Device(d)
    }
}

impl Device {
    /// [`crate::Device::open`]. An RTL-SDR or a HackRF is opened through
    /// its driver's blocking open, bounded by `sdr_core::open_with_deadline`
    /// (a device that enumerates but never answers fails within seconds); a
    /// USRP is not bounded (its first firmware and FPGA load takes seconds).
    #[cfg_attr(not(any(feature = "rtl", feature = "hackrf")), allow(unused_variables))]
    pub fn open(spec: &str) -> Result<Device> {
        let (scheme, rest) = spec.split_once(':').unwrap_or((spec, ""));
        match scheme {
            #[cfg(feature = "rtl")]
            "rtl" => sdr_rtl::blocking::Rtl::open(rest)
                .map(|r| Device(crate::Device::Rtl(r.into_async()))),
            #[cfg(feature = "hackrf")]
            "hackrf" => sdr_hackrf::blocking::HackRf::open(rest)
                .map(|h| Device(crate::Device::HackRf(h.into_async()))),
            _ => block_on(crate::Device::open(spec)).map(Device),
        }
    }

    /// The async radio.
    pub fn into_async(self) -> crate::Device {
        self.0
    }

    pub fn caps(&self) -> Caps {
        self.0.caps()
    }

    pub fn label(&self) -> String {
        self.0.label()
    }

    pub fn set_rate(&mut self, sps: f64) -> Result<()> {
        block_on(self.0.set_rate(sps))
    }

    pub fn rate(&self) -> Result<f64> {
        self.0.rate()
    }

    pub fn set_freq(&mut self, req: &TuneRequest) -> Result<TuneResult> {
        block_on(self.0.set_freq(req))
    }

    pub fn freq(&self) -> Result<f64> {
        self.0.freq()
    }

    pub fn set_gain(&mut self, db: f64) -> Result<()> {
        block_on(self.0.set_gain(db))
    }

    pub fn gain(&self) -> Result<f64> {
        self.0.gain()
    }

    pub fn stages(&self) -> Vec<Stage> {
        self.0.stages()
    }

    pub fn stage_levels_db(&self, stage: Stage) -> Option<Vec<f64>> {
        self.0.stage_levels_db(stage)
    }

    pub fn stage_readings(&self) -> Result<Vec<StageReading>> {
        self.0.stage_readings()
    }

    pub fn set_stage_level(&mut self, stage: Stage, level: usize) -> Result<()> {
        block_on(self.0.set_stage_level(stage, level))
    }

    pub fn set_bandwidth(&mut self, hz: f64) -> Result<()> {
        block_on(self.0.set_bandwidth(hz))
    }

    /// [`crate::Device::into_stream`].
    pub fn into_stream(self, cfg: &StreamConfig) -> core::result::Result<Stream, StreamError> {
        block_on(self.0.into_stream(cfg))
            .map(Stream)
            .map_err(StreamError::from)
    }

    /// [`crate::Device::into_tx_stream`].
    pub fn into_tx_stream(self, cfg: &StreamConfig) -> core::result::Result<TxStream, StreamError> {
        block_on(self.0.into_tx_stream(cfg))
            .map(TxStream)
            .map_err(StreamError::from)
    }

    /// The RTL-SDR underneath (async; `sdr_core::block_on` runs a call).
    #[cfg(feature = "rtl")]
    pub fn as_rtl(&mut self) -> Option<&mut sdr_rtl::Rtl> {
        self.0.as_rtl()
    }

    /// The HackRF underneath (async; `sdr_core::block_on` runs a call).
    #[cfg(feature = "hackrf")]
    pub fn as_hackrf(&mut self) -> Option<&mut sdr_hackrf::HackRf> {
        self.0.as_hackrf()
    }

    /// The USRP underneath.
    #[cfg(feature = "uhd")]
    pub fn as_uhd(&mut self) -> Option<&mut sdr_uhd::Usrp> {
        self.0.as_uhd()
    }
}

/// A stream could not be made; the radio is handed back.
#[derive(Debug)]
pub struct StreamError {
    pub device: Box<Device>,
    pub error: Error,
}

impl From<crate::StreamError> for StreamError {
    fn from(e: crate::StreamError) -> StreamError {
        StreamError {
            device: Box::new(Device(*e.device)),
            error: e.error,
        }
    }
}

impl core::fmt::Display for StreamError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

impl core::error::Error for StreamError {}

/// A finished stream ([`crate::Finished`]): the radio back.
pub struct Finished {
    pub device: Device,
    pub report: StreamReport,
    pub tail: Vec<Iq>,
}

/// A finished transmission ([`crate::TxFinished`]): the radio back.
pub struct TxFinished {
    pub device: Device,
    pub report: TxReport,
}

/// [`crate::Stream`], blocking.
pub struct Stream(crate::Stream);

/// [`crate::ReadHalf`], blocking.
pub struct ReadHalf(crate::ReadHalf);

/// [`crate::Control`], blocking. Cheap to clone.
#[derive(Clone)]
pub struct Control(crate::Control);

/// [`crate::TxStream`], blocking.
pub struct TxStream(crate::TxStream);

/// [`crate::WriteHalf`], blocking.
pub struct WriteHalf(crate::WriteHalf);

/// [`crate::TxControl`], blocking. Cheap to clone.
#[derive(Clone)]
pub struct TxControl(crate::TxControl);

macro_rules! reports {
    ($t:ident) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                self.0.rate()
            }

            pub fn freq(&self) -> Result<f64> {
                self.0.freq()
            }

            pub fn gain(&self) -> f64 {
                self.0.gain()
            }

            pub fn stage_readings(&self) -> Vec<StageReading> {
                self.0.stage_readings()
            }

            pub fn label(&self) -> String {
                self.0.label()
            }

            pub fn delivered(&self) -> u64 {
                self.0.delivered()
            }

            pub fn breaks(&self) -> Vec<Break> {
                self.0.breaks()
            }

            pub fn max_num_samps(&self) -> usize {
                self.0.max_num_samps()
            }
        }
    };
}

macro_rules! changes {
    ($t:ident) => {
        impl $t {
            pub fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Break)> {
                block_on(self.0.retune(req))
            }

            pub fn set_gain(&self, db: f64) -> Result<Break> {
                block_on(self.0.set_gain(db))
            }

            pub fn set_stage_level(&self, stage: Stage, level: usize) -> Result<Break> {
                block_on(self.0.set_stage_level(stage, level))
            }

            pub fn set_bandwidth(&self, hz: f64) -> Result<Break> {
                block_on(self.0.set_bandwidth(hz))
            }
        }
    };
}

macro_rules! reads {
    ($t:ident) => {
        impl $t {
            pub fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
                block_on(self.0.recv_into(buf, timeout_secs))
            }

            /// The radio's own wire bytes ([`crate::Stream::read_native`]).
            pub fn read_native(&mut self, buf: &mut [u8]) -> Result<usize> {
                block_on(self.0.read_native(buf))
            }

            pub fn wire_format(&self) -> WireFormat {
                self.0.wire_format()
            }

            pub fn start(&mut self) -> Result<()> {
                block_on(self.0.start())
            }
        }

        /// The radio's own wire bytes ([`crate::WireFormat`]). Blocks until
        /// at least one byte; end of file once the stream has finished.
        impl io::Read for $t {
            fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                block_on(self.0.read_native(buf)).map_err(io::Error::other)
            }
        }

        impl RxSource for $t {
            fn max_num_samps(&self) -> usize {
                self.0.max_num_samps()
            }

            fn start(&mut self) -> Result<()> {
                block_on(self.0.start())
            }

            fn recv_into(&mut self, buf: &mut [Iq], timeout_secs: f64) -> Result<RecvInfo> {
                block_on(self.0.recv_into(buf, timeout_secs))
            }

            fn setup_worker_thread(&self) {
                elevate_current_thread();
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
    /// The async stream.
    pub fn into_async(self) -> crate::Stream {
        self.0
    }

    pub fn split(self) -> (ReadHalf, Control) {
        let (r, c) = self.0.split();
        (ReadHalf(r), Control(c))
    }

    // The halves come back whole when they do not match; reuniting is
    // rare, so their size there costs nothing.
    #[allow(clippy::result_large_err)]
    pub fn reunite(
        read: ReadHalf,
        control: Control,
    ) -> core::result::Result<Stream, (ReadHalf, Control)> {
        crate::Stream::reunite(read.0, control.0)
            .map(Stream)
            .map_err(|(r, c)| (ReadHalf(r), Control(c)))
    }

    pub fn finish(self) -> Result<Finished> {
        block_on(self.0.finish()).map(|f| Finished {
            device: Device(f.device),
            report: f.report,
            tail: f.tail,
        })
    }
}

macro_rules! tx_reports {
    ($t:ident) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                self.0.rate()
            }

            pub fn written(&self) -> u64 {
                self.0.written()
            }

            pub fn label(&self) -> String {
                self.0.label()
            }
        }
    };
}

macro_rules! tx_writes {
    ($t:ident) => {
        impl $t {
            pub fn send(&mut self, iq: &[Iq]) -> Result<()> {
                block_on(self.0.send(iq))
            }

            pub fn samples_per_transfer(&self) -> usize {
                self.0.samples_per_transfer()
            }
        }
    };
}

macro_rules! tx_changes {
    ($t:ident) => {
        impl $t {
            pub fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Option<Break>)> {
                block_on(self.0.retune(req))
            }
        }
    };
}

tx_reports!(TxStream);
tx_reports!(WriteHalf);
tx_reports!(TxControl);
tx_writes!(TxStream);
tx_writes!(WriteHalf);
tx_changes!(TxStream);
tx_changes!(TxControl);

impl TxStream {
    /// The async transmission.
    pub fn into_async(self) -> crate::TxStream {
        self.0
    }

    pub fn split(self) -> (WriteHalf, TxControl) {
        let (w, c) = self.0.split();
        (WriteHalf(w), TxControl(c))
    }

    pub fn reunite(
        write: WriteHalf,
        control: TxControl,
    ) -> core::result::Result<TxStream, (WriteHalf, TxControl)> {
        crate::TxStream::reunite(write.0, control.0)
            .map(TxStream)
            .map_err(|(w, c)| (WriteHalf(w), TxControl(c)))
    }

    pub fn finish(self) -> Result<TxFinished> {
        block_on(self.0.finish()).map(|f| TxFinished {
            device: Device(f.device),
            report: f.report,
        })
    }
}
