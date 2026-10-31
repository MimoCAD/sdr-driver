//! One API over the sdr-* radio drivers: an RTL-SDR (sdr-rtl), a HackRF
//! (sdr-hackrf) and a USRP B2xx (sdr-uhd, over libuhd) behind one
//! [`Device`], one [`Stream`] of [`Iq`] samples and one [`TxStream`].
//!
//! - **Finding and opening.** [`list`] names every attached radio of the
//!   compiled-in drivers; [`Device::open`] takes the spec it reports
//!   (`rtl:serial=…`, `hackrf:`, `uhd:type=b200`, `""` for the first
//!   found); in a browser [`Device::request`] asks the user.
//! - **Receiving.** [`Device::into_stream`] makes the radio a [`Stream`]:
//!   it owns the radio until [`Stream::finish`] hands it back with a
//!   [`StreamReport`]. [`Stream::split`] gives a [`ReadHalf`] and a
//!   [`Control`]; every change the control makes while samples flow comes
//!   back as a [`Break`], with one meaning on every radio: before
//!   `at_sample` the old configuration, from `settled_at` the new.
//! - **Loss.** [`StreamReport::lost`] is samples lost without meaning to,
//!   as each radio counts them, with how sure the count is ([`Loss`]): the
//!   RTL2832U's dropped-block counter is exact (unless it stopped at 255),
//!   the HackRF's firmware keeps the longest shortfall (at least), a USRP's
//!   timestamps give the gap exactly. A change's planned gap — an
//!   RTL-SDR stops sampling while it retunes — is [`Break::gap`], not loss.
//! - **Transmitting.** [`Device::into_tx_stream`] (HackRF, USRP; see
//!   [`Caps::transmit`]).
//! - **Each radio's own.** [`Device::as_rtl`] and its siblings reach the
//!   driver underneath (an EEPROM, the M0 counters, an antenna port);
//!   [`Stream::read_native`] reads the radio's own wire format
//!   ([`WireFormat`]) without conversion.
//!
//! Every method that waits is async — in a browser it suspends; natively
//! it completes in place — and [`blocking`] wraps each one.
//!
//! Features: `rtl` and `hackrf` (default; pure Rust, also over WebUSB),
//! `uhd` (links libuhd; native only).

#[cfg(not(any(feature = "rtl", feature = "hackrf", feature = "uhd")))]
compile_error!("sdr-driver needs at least one backend feature: rtl, hackrf or uhd");

#[cfg(not(target_arch = "wasm32"))]
pub mod blocking;
mod device;
mod stream;
mod tx;
#[cfg(feature = "uhd")]
mod uhd;

pub use device::{Caps, Device, StreamConfig, StreamError, WireFormat, list};
pub use stream::{BackendReport, Break, Control, Finished, Loss, ReadHalf, Stream, StreamReport};
pub use tx::{TxBackendReport, TxControl, TxFinished, TxReport, TxStream, WriteHalf};
#[cfg(feature = "uhd")]
pub use uhd::{
    Uhd, UhdControl, UhdReadHalf, UhdReport, UhdStream, UhdTxControl, UhdTxStream, UhdWriteHalf,
};

pub use sdr_core::gain::{Stage, StageReading};
pub use sdr_core::{
    Backend, DeviceDesc, Error, Iq, OtwFormat, RecvInfo, Result, TunePolicy, TuneRequest,
    TuneResult,
};

/// The drivers underneath, for what only one radio has.
#[cfg(feature = "hackrf")]
pub use sdr_hackrf as hackrf;
#[cfg(feature = "rtl")]
pub use sdr_rtl as rtl;
#[cfg(feature = "uhd")]
pub use sdr_uhd as usrp;

/// The error a backend this build left out answers with.
#[cfg(not(all(feature = "rtl", feature = "hackrf", feature = "uhd")))]
fn not_built(backend: &'static str) -> Error {
    Error::Unsupported {
        backend,
        what: format!("sdr-driver built without the `{backend}` feature"),
    }
}
