# sdr-driver

One API over the sdr-* radio drivers: an RTL-SDR ([sdr-rtl]), a HackRF
([sdr-hackrf]) and a USRP B2xx ([sdr-uhd], over libuhd) behind one
`Device`, one `Stream` of `Iq` samples and one `TxStream`.

```rust
use sdr_driver::blocking::Device;
use sdr_driver::{Iq, StreamConfig, TuneRequest};

let mut radio = Device::open("")?;            // the first radio found; or "rtl:", "hackrf:", "uhd:type=b200"
radio.set_rate(2.4e6)?;
radio.set_freq(&TuneRequest::auto(100.1e6))?;
let (mut reader, control) = radio.into_stream(&StreamConfig::default()).map_err(|e| e.error)?.split();
let mut buf = vec![Iq::default(); reader.max_num_samps()];
let info = reader.recv_into(&mut buf, 1.0)?;   // on this thread…
let (_, brk) = control.retune(&TuneRequest::auto(100.3e6))?; // …changes from another
// brk.at_sample: the last sample of the old tuning is before it;
// brk.settled_at: every sample from it is the new tuning.
```

- **Finding.** `list()` names every attached radio of the compiled-in
  drivers, best first (USRP, HackRF, RTL-SDR); each `DeviceDesc.spec`
  opens it. In a browser, `Device::request()` opens the WebUSB chooser.
- **Receiving.** A radio becomes a `Stream` (`into_stream`) that owns it
  until `finish` hands it back with a `StreamReport`. `split` gives a
  `ReadHalf` and a `Control` for two threads or tasks.
- **Every change placed.** Each change made while receiving returns a
  `Break` with one meaning on every radio: before `at_sample` the old
  configuration, from `settled_at` the new. An RTL-SDR changes inside a
  pause: it stops sampling for `Break::gap` (~60 ms), so a consumer keeping
  time adds it; a HackRF and a USRP keep streaming (`gap` zero) and the two
  bracket the request, read from the radio's own count (the HackRF's M0
  byte count, the USRP's clock).
- **Loss as the radio counts it.** `StreamReport::lost` is samples lost
  without meaning to, with how sure the count is (`Loss::None`, `Exact`,
  `AtLeast`): an RTL-SDR's dropped-block counter, a HackRF's longest
  shortfall (a floor), a USRP's timestamps. A change's planned gap is not
  loss; it is in the `Break`.
- **Transmitting.** A HackRF or a USRP becomes a `TxStream`
  (`into_tx_stream`; `Caps::transmit` says which can).
- **Each radio's own.** `as_rtl()`, `as_hackrf()`, `as_uhd()` reach the
  driver underneath (an RTL's EEPROM, a HackRF's amp and counters, a
  USRP's antennas and clock); `read_native` reads the radio's own wire
  format (`WireFormat`) without conversion, and the blocking streams are
  `std::io::Read` over it.

Async at its core (WebUSB in a browser; natively every call completes in
place), with `blocking` wrapping every call.

## Features

| Feature | Radio | Needs |
|---|---|---|
| `rtl` (default) | RTL2832U + R820T/R820T2 | nothing (pure Rust, nusb; also WebUSB) |
| `hackrf` (default) | HackRF | nothing (pure Rust, nusb; also WebUSB) |
| `uhd` | USRP B200, B210, B205mini, B206mini | libuhd ([sdr-uhd]'s README builds it) |

`rtl` and `hackrf` are pure Rust under this crate's licence. **`uhd` links
libuhd, which is GPL-3.0-or-later:** a binary built with it and given to
anyone else comes under GPLv3 (see [sdr-uhd]'s README). That is why it is
off by default.

## Measured

On a HackRF Pro and a USRP B206mini cabled through a 20 dB pad
(`examples/drivercheck.rs`, `examples/retunecheck.rs`): each radio received
2 s at 2 Msps through three retunes from a second thread with nothing lost;
each one's tone reached the other through `TxStream` and `Stream`; and a
tone into the USRP placed ten of ten of its retunes between the `Break`'s
`at_sample` and `settled_at` at 2 and 4 Msps (within one 32-sample
window). sdr-hackrf's and sdr-rtl's own records measure theirs.

A USRP's `set_rate` pins the master clock at twice the rate: with the clock
equal to the rate (decimation 1, which UHD's own choice can land on) the
AD9364 puts sidebands ±270–310 kHz around every strong carrier. On the
bench (`examples/spurcheck.rs`, a HackRF tone into the B206mini) decimation
1 showed −35 dBc at 19.2 and 24 Msps, 5.5 dB above the floor; decimation 2
nothing above it. Above 30.72 Msps the clock (at most 61.44 MHz) cannot be
doubled.

License: MIT OR Apache-2.0.

[sdr-rtl]: https://crates.io/crates/sdr-rtl
[sdr-hackrf]: https://crates.io/crates/sdr-hackrf
[sdr-uhd]: https://crates.io/crates/sdr-uhd
