//! Where a retune lands on any radio, against a tone another radio sends,
//! through sdr-driver's common API: `tx` keys a tone at `TONE_HZ` (its
//! transmit gain low, from the driver underneath), `rx` receives while a
//! second thread retunes it every 200 ms between 300.0 and 300.1 MHz (the
//! tone at +300 and +200 kHz). Afterwards, in windows of `WINDOW` samples,
//! the first where the new offset outweighs the old (the takeover) is set
//! against the [`Break`]'s `at_sample` and `settled_at`, and from it the
//! frequency in `FREQ_WINDOW` windows until it stays within `SETTLED_HZ`.
//!
//!   cargo run --release -p sdr-driver --features uhd --example retunecheck -- --cabled <tx spec> <rx spec> [rate]
//!
//! `--cabled` is the operator's statement that the two are cabled through
//! fixed attenuation and nothing radiates. One JSON line per break, then a
//! summary.

use std::f64::consts::TAU;
use std::thread;
use std::time::Duration;

use sdr_driver::blocking::{Device, ReadHalf, Stream};
use sdr_driver::{Backend, Break, Iq, StreamConfig, TuneRequest};

const TONE_HZ: f64 = 300.3e6;
const RF: [f64; 2] = [300.0e6, 300.1e6];
const RETUNES: usize = 10;
const INTERVAL: Duration = Duration::from_millis(200);
const RX_GAIN_DB: f64 = 30.0;
/// The transmitter's gain: a HackRF's transmit VGA (amp off), a USRP's
/// transmit gain. Through a 20 dB pad, tens of dB below either receiver's
/// limit.
const HACKRF_TXVGA_DB: u16 = 30;
const USRP_TX_GAIN_DB: f64 = 60.0;
const BACKOFF_DB: f64 = -20.0;
const WINDOW: usize = 32;
const FREQ_WINDOW: usize = 256;
const SETTLED_HZ: f64 = 500.0;
const SETTLED_FOR: usize = 8;
/// How far past `settled_at` the settling is looked for, samples.
const SEARCH_AFTER: usize = 400_000;

fn corr(x: &[Iq], n0: usize, f: f64, rate: f64) -> (f64, f64) {
    let (mut re, mut im) = (0.0, 0.0);
    for (k, s) in x.iter().enumerate() {
        let ph = -TAU * f * (n0 + k) as f64 / rate;
        let (c, sn) = (ph.cos(), ph.sin());
        re += s.i as f64 * c - s.q as f64 * sn;
        im += s.i as f64 * sn + s.q as f64 * c;
    }
    (re, im)
}

/// The takeover window and the settled sample after break `k`, if found.
fn place(x: &[Iq], b: &Break, k: usize, rate: f64) -> (Option<usize>, Option<usize>) {
    let old_off = TONE_HZ - RF[k % 2];
    let new_off = TONE_HZ - RF[(k + 1) % 2];
    let n_windows = x.len() / WINDOW;
    let from = (b.at_sample as usize / WINDOW).saturating_sub(64);
    let to = ((b.settled_at as usize + SEARCH_AFTER) / WINDOW).min(n_windows);
    let mag = |w: usize, f: f64| {
        let (re, im) = corr(&x[w * WINDOW..(w + 1) * WINDOW], w * WINDOW, f, rate);
        re.hypot(im)
    };
    let Some(t) = (from..to).find(|&w| mag(w, new_off) > mag(w, old_off)) else {
        return (None, None);
    };
    let fw = |j: usize| {
        let n0 = t * WINDOW + j * FREQ_WINDOW;
        (n0 + FREQ_WINDOW <= x.len()).then(|| corr(&x[n0..n0 + FREQ_WINDOW], n0, new_off, rate))
    };
    let (mut prev, mut run) = (fw(0), 0);
    for j in 1.. {
        let (Some(p), Some(c)) = (prev, fw(j)) else {
            break;
        };
        let err_hz =
            (c.1 * p.0 - c.0 * p.1).atan2(c.0 * p.0 + c.1 * p.1) / TAU * rate / FREQ_WINDOW as f64;
        prev = Some(c);
        run = if err_hz.abs() <= SETTLED_HZ {
            run + 1
        } else {
            0
        };
        if run == SETTLED_FOR {
            return (
                Some(t * WINDOW),
                Some(t * WINDOW + (j - SETTLED_FOR) * FREQ_WINDOW),
            );
        }
        if t * WINDOW + j * FREQ_WINDOW > to * WINDOW {
            break;
        }
    }
    (Some(t * WINDOW), None)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("--cabled") || args.len() < 3 {
        eprintln!(
            "usage: retunecheck --cabled <tx spec> <rx spec> [rate]   (--cabled: the two are cabled through attenuation)"
        );
        std::process::exit(2);
    }
    let rate: f64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(2e6);

    // The transmitter: the tone at TONE_HZ, keyed and fed from a thread.
    let mut tx_dev = Device::open(&args[1])?;
    tx_dev.set_rate(rate)?;
    let lo = TONE_HZ - 250e3;
    match tx_dev.caps().backend {
        Backend::HackRf => {
            let h = tx_dev.as_hackrf().expect("a HackRF");
            sdr_core::block_on(h.set_amp(false))?;
            sdr_core::block_on(h.set_txvga_gain(HACKRF_TXVGA_DB))?;
            sdr_core::block_on(h.set_freq(&TuneRequest::auto(lo)))?;
        }
        #[cfg(feature = "uhd")]
        Backend::Usrp => {
            let u = tx_dev.as_uhd().expect("a USRP");
            u.set_tx_rate(rate, 0)?;
            u.set_tx_antenna("TX/RX", 0)?;
            u.set_tx_gain(USRP_TX_GAIN_DB, 0)?;
            u.set_tx_freq(&TuneRequest::auto(lo), 0)?;
        }
        other => return Err(format!("{other:?} does not transmit").into()),
    }
    let mut tx = tx_dev
        .into_tx_stream(&StreamConfig::default())
        .map_err(|e| e.error)?;
    let keyed = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let feeding = std::sync::Arc::clone(&keyed);
    let feeder = thread::spawn(
        move || -> sdr_driver::Result<sdr_driver::blocking::TxStream> {
            let amplitude = 32767.0 * 10f64.powf(BACKOFF_DB / 20.0);
            let step = TAU * 250e3 / rate;
            let mut phase = 0.0f64;
            let mut chunk = vec![Iq::default(); tx.samples_per_transfer()];
            while feeding.load(std::sync::atomic::Ordering::Relaxed) {
                for s in chunk.iter_mut() {
                    *s = Iq {
                        i: (amplitude * phase.cos()) as i16,
                        q: (amplitude * phase.sin()) as i16,
                    };
                    phase = (phase + step) % TAU;
                }
                tx.send(&chunk)?;
            }
            Ok(tx)
        },
    );
    thread::sleep(Duration::from_millis(500));

    // The receiver.
    let mut rx_dev = Device::open(&args[2])?;
    rx_dev.set_rate(rate)?;
    rx_dev.set_freq(&TuneRequest::auto(RF[0]))?;
    rx_dev.set_gain(RX_GAIN_DB)?;
    #[cfg(feature = "uhd")]
    if let Some(u) = rx_dev.as_uhd() {
        u.set_rx_antenna("TX/RX", 0)?;
    }
    let label = rx_dev.label();
    let (mut reader, control) = rx_dev
        .into_stream(&StreamConfig::default())
        .map_err(|e| e.error)?
        .split();
    let want = (rate * ((RETUNES + 2) as f64 * INTERVAL.as_secs_f64())) as usize;
    let read_thread = thread::spawn(move || -> sdr_driver::Result<(Vec<Iq>, ReadHalf)> {
        let mut all = Vec::with_capacity(want);
        let mut buf = vec![Iq::default(); reader.max_num_samps()];
        while all.len() < want {
            let n = reader.recv_into(&mut buf, 0.5)?.n_samples;
            all.extend_from_slice(&buf[..n]);
        }
        Ok((all, reader))
    });
    let mut breaks = Vec::new();
    for k in 0..RETUNES {
        thread::sleep(INTERVAL);
        breaks.push(control.retune(&TuneRequest::auto(RF[(k + 1) % 2]))?.1);
    }
    let (x, reader) = read_thread.join().expect("the reading thread")?;
    let report = Stream::reunite(reader, control)
        .map_err(|_| "reunite")?
        .finish()?
        .report;
    keyed.store(false, std::sync::atomic::Ordering::Relaxed);
    feeder.join().expect("the feeding thread")?.finish()?;

    let (mut after_at, mut before_settled, mut settled) = (Vec::new(), Vec::new(), Vec::new());
    for (k, b) in breaks.iter().enumerate() {
        let (t, s) = place(&x, b, k, rate);
        if let Some(t) = t {
            after_at.push(t as i64 - b.at_sample as i64);
            before_settled.push(t as i64 - b.settled_at as i64);
        }
        if let Some(s) = s {
            settled.push(s as i64 - b.settled_at as i64);
        }
        println!(
            "{{\"break\":{k},\"at_sample\":{},\"settled_at\":{},\"takeover\":{t:?},\"settled\":{s:?}}}",
            b.at_sample, b.settled_at
        );
    }
    let span = |v: &mut Vec<i64>| {
        v.sort();
        (v.first().copied(), v.last().copied())
    };
    println!(
        "{{\"summary\":true,\"rx\":\"{label}\",\"rate\":{rate},\"breaks\":{},\"placed\":{},\"takeover_minus_at_sample\":{:?},\"takeover_minus_settled_at\":{:?},\"settled\":{},\"settled_minus_settled_at\":{:?},\"lost\":\"{:?}\"}}",
        breaks.len(),
        after_at.len(),
        span(&mut after_at),
        span(&mut before_settled),
        settled.len(),
        span(&mut settled),
        report.lost
    );
    Ok(())
}
