//! The radio transmitting: one enum over each driver's transmit stream.
//! (Built with no radio that transmits — `rtl` alone — the enums hold only
//! a variant that can never be made.)

use sdr_core::{Iq, Result, TuneRequest, TuneResult};

use crate::device::Device;
use crate::stream::Break;

/// What a finished transmission did.
#[derive(Clone, Debug, PartialEq)]
pub struct TxReport {
    /// Samples written.
    pub samples: u64,
    /// Times the radio found nothing to send while samples were still to
    /// come; `None` where the radio does not say.
    pub underruns: Option<u32>,
    /// The radio had sent every sample before it stopped; `None` where it
    /// does not say.
    pub sent_all: Option<bool>,
    /// The driver's own report.
    pub detail: TxBackendReport,
}

/// A driver's own transmit report.
#[derive(Clone, Debug, PartialEq)]
pub enum TxBackendReport {
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::TxReport),
    /// A USRP reports nothing beyond the samples written.
    #[cfg(feature = "uhd")]
    Uhd,
}

/// A finished transmission: the radio back and the report.
pub struct TxFinished {
    pub device: Device,
    pub report: TxReport,
}

/// The radio transmitting.
pub enum TxStream {
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::TxStream),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdTxStream),
    #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
    #[doc(hidden)]
    Never(core::convert::Infallible),
}

/// The writing half of a [`TxStream`] ([`TxStream::split`]).
pub enum WriteHalf {
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::WriteHalf),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdWriteHalf),
    #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
    #[doc(hidden)]
    Never(core::convert::Infallible),
}

/// The changing half of a [`TxStream`] ([`TxStream::split`]). Cheap to
/// clone.
#[derive(Clone)]
pub enum TxControl {
    #[cfg(feature = "hackrf")]
    HackRf(sdr_hackrf::TxControl),
    #[cfg(feature = "uhd")]
    Uhd(crate::uhd::UhdTxControl),
    #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
    #[doc(hidden)]
    Never(core::convert::Infallible),
}

/// Each variant's `$s` bound to the driver's own value.
macro_rules! tx_on {
    ($e:expr, $ty:ident, $s:ident => $body:expr) => {
        match $e {
            #[cfg(feature = "hackrf")]
            $ty::HackRf($s) => $body,
            #[cfg(feature = "uhd")]
            $ty::Uhd($s) => $body,
            #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
            $ty::Never(n) => match *n {},
        }
    };
}

/// What every transmit handle reports.
macro_rules! tx_reports {
    ($t:ident) => {
        impl $t {
            pub fn rate(&self) -> f64 {
                tx_on!(self, $t, s => s.rate())
            }

            /// Samples written so far.
            pub fn written(&self) -> u64 {
                tx_on!(self, $t, s => s.written())
            }

            /// The radio's label.
            pub fn label(&self) -> String {
                tx_on!(self, $t, s => s.label())
            }
        }
    };
}

/// What changes the radio while it transmits.
macro_rules! tx_changes {
    ($t:ident) => {
        impl $t {
            /// Retune while transmitting; the [`Break`] (in samples written)
            /// where the radio can place it.
            #[cfg_attr(not(any(feature = "hackrf", feature = "uhd")), allow(unused_variables))]
            pub async fn retune(&self, req: &TuneRequest) -> Result<(TuneResult, Option<Break>)> {
                match self {
                    #[cfg(feature = "hackrf")]
                    $t::HackRf(s) => s.retune(req).await.map(|(t, b)| (t, Some(b.into()))),
                    #[cfg(feature = "uhd")]
                    $t::Uhd(s) => s.retune(req).await.map(|t| (t, None)),
                    #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
                    $t::Never(n) => match *n {},
                }
            }
        }
    };
}

/// What writes the samples.
macro_rules! tx_writes {
    ($t:ident) => {
        impl $t {
            /// Samples to send: waits only while the radio's queue is full,
            /// which paces the writer at the sample rate.
            #[cfg_attr(not(any(feature = "hackrf", feature = "uhd")), allow(unused_variables))]
            pub async fn send(&mut self, iq: &[Iq]) -> Result<()> {
                tx_on!(self, $t, s => s.send(iq).await)
            }

            /// Samples one transfer carries: queue at least a few before
            /// expecting a clean start.
            pub fn samples_per_transfer(&self) -> usize {
                tx_on!(self, $t, s => s.samples_per_transfer())
            }
        }
    };
}

tx_reports!(TxStream);
tx_reports!(WriteHalf);
tx_reports!(TxControl);
tx_changes!(TxStream);
tx_changes!(TxControl);
tx_writes!(TxStream);
tx_writes!(WriteHalf);

impl TxStream {
    /// The two halves, each usable on its own thread or task.
    pub fn split(self) -> (WriteHalf, TxControl) {
        match self {
            #[cfg(feature = "hackrf")]
            TxStream::HackRf(s) => {
                let (w, c) = s.split();
                (WriteHalf::HackRf(w), TxControl::HackRf(c))
            }
            #[cfg(feature = "uhd")]
            TxStream::Uhd(s) => {
                let (w, c) = s.split();
                (WriteHalf::Uhd(w), TxControl::Uhd(c))
            }
            #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
            TxStream::Never(n) => match n {},
        }
    }

    /// The halves of one transmission joined again.
    pub fn reunite(
        write: WriteHalf,
        control: TxControl,
    ) -> core::result::Result<TxStream, (WriteHalf, TxControl)> {
        match (write, control) {
            #[cfg(feature = "hackrf")]
            (WriteHalf::HackRf(w), TxControl::HackRf(c)) => sdr_hackrf::TxStream::reunite(w, c)
                .map(TxStream::HackRf)
                .map_err(|(w, c)| (WriteHalf::HackRf(w), TxControl::HackRf(c))),
            #[cfg(feature = "uhd")]
            (WriteHalf::Uhd(w), TxControl::Uhd(c)) => crate::uhd::UhdTxStream::reunite(w, c)
                .map(TxStream::Uhd)
                .map_err(|(w, c)| (WriteHalf::Uhd(w), TxControl::Uhd(c))),
            #[allow(unreachable_patterns)]
            other => Err(other),
        }
    }

    /// Send what is queued, stop, and report: the radio back.
    pub async fn finish(self) -> Result<TxFinished> {
        match self {
            #[cfg(feature = "hackrf")]
            TxStream::HackRf(s) => {
                let f = s.finish().await?;
                let r = &f.report;
                let report = TxReport {
                    samples: r.samples,
                    underruns: Some(r.underruns),
                    sent_all: Some(r.sent_all),
                    detail: TxBackendReport::HackRf(f.report.clone()),
                };
                Ok(TxFinished {
                    device: Device::HackRf(f.hackrf),
                    report,
                })
            }
            #[cfg(feature = "uhd")]
            TxStream::Uhd(s) => {
                let (u, samples) = s.finish().await?;
                Ok(TxFinished {
                    device: Device::Uhd(u),
                    report: TxReport {
                        samples,
                        underruns: None,
                        sent_all: None,
                        detail: TxBackendReport::Uhd,
                    },
                })
            }
            #[cfg(not(any(feature = "hackrf", feature = "uhd")))]
            TxStream::Never(n) => match n {},
        }
    }

    /// The HackRF's own transmit stream, if it is one.
    #[cfg(feature = "hackrf")]
    pub fn as_hackrf(&mut self) -> Option<&mut sdr_hackrf::TxStream> {
        match self {
            TxStream::HackRf(s) => Some(s),
            #[allow(unreachable_patterns)]
            _ => None,
        }
    }
}
