//! The B2xx decimation-1 spurs, on the bench: a HackRF sends a strong tone
//! into a USRP (cabled through attenuation), and the USRP receives it with
//! its master clock (a) forced equal to the sample rate — decimation 1 — and
//! (b) as sdr-driver's `set_rate` sets it, at twice the rate. In each, the
//! spectrum around the tone is read packet by packet (each receive is
//! contiguous even when the link overflows), and the strongest signal in the
//! bands ±`SPUR_BAND_HZ` from the tone is set against the tone and against
//! the floor further out.
//!
//!   cargo run --release -p sdr-driver --features uhd --example spurcheck -- --cabled <hackrf spec> <usrp spec> [rate …]

use std::f64::consts::TAU;
use std::thread;
use std::time::Duration;

use sdr_driver::blocking::Device;
use sdr_driver::{Iq, StreamConfig, TuneRequest};

const RX_HZ: f64 = 300e6;
/// The tone, this far above the USRP's centre.
const TONE_OFFSET_HZ: f64 = 1e6;
const HACKRF_TXVGA_DB: u16 = 30;
const RX_GAIN_DB: f64 = 30.0;
/// Where the decimation-1 spurs were measured: ±270–310 kHz from a carrier
/// (searched a little wider).
const SPUR_BAND_HZ: (f64, f64) = (260e3, 320e3);
/// The floor: further from the tone than the spurs.
const FLOOR_BAND_HZ: (f64, f64) = (400e3, 600e3);
const STEP_HZ: f64 = 2e3;
/// Receives averaged per configuration.
const RECEIVES: usize = 60;

/// |DFT at f|² over one contiguous receive, Hann-windowed (an unwindowed
/// receive leaks the tone ~40 dB down across the spur bands, which hid them).
fn power(x: &[Iq], f: f64, rate: f64) -> f64 {
    let n_all = x.len() as f64;
    let (mut re, mut im) = (0.0, 0.0);
    for (n, s) in x.iter().enumerate() {
        let w = 0.5 - 0.5 * (TAU * n as f64 / n_all).cos();
        let ph = -TAU * f * n as f64 / rate;
        let (c, sn) = (ph.cos(), ph.sin());
        re += w * (s.i as f64 * c - s.q as f64 * sn);
        im += w * (s.i as f64 * sn + s.q as f64 * c);
    }
    re * re + im * im
}

fn db(p: f64) -> f64 {
    10.0 * (p + 1e-12).log10()
}

/// The offsets from the tone in a band, both sides.
fn band(b: (f64, f64)) -> Vec<f64> {
    let mut v = Vec::new();
    let mut o = b.0;
    while o <= b.1 {
        v.push(o);
        v.push(-o);
        o += STEP_HZ;
    }
    v
}

fn measure(usrp_spec: &str, rate: f64, decim1: bool) -> Result<String, Box<dyn std::error::Error>> {
    let mut d = Device::open(usrp_spec)?;
    d.set_rate(rate)?; // the master clock at twice the rate
    if decim1 {
        let u = d.as_uhd().expect("a USRP");
        u.set_master_clock_rate(rate)?;
        u.set_rx_rate(rate, 0)?;
    }
    let mcr = d.as_uhd().expect("a USRP").master_clock_rate()?;
    d.set_freq(&TuneRequest::auto(RX_HZ))?;
    d.set_gain(RX_GAIN_DB)?;
    d.as_uhd().expect("a USRP").set_rx_antenna("TX/RX", 0)?;
    let mut s = d
        .into_stream(&StreamConfig::default())
        .map_err(|e| e.error)?;
    let mut buf = vec![Iq::default(); s.max_num_samps()];
    let (spur_band, floor_band) = (band(SPUR_BAND_HZ), band(FLOOR_BAND_HZ));
    let mut tone = 0.0;
    let mut spur = vec![0.0; spur_band.len()];
    let mut floor = 0.0;
    let mut used = 0;
    // Settle, then average full receives.
    for _ in 0..20 {
        s.recv_into(&mut buf, 0.5)?;
    }
    while used < RECEIVES {
        let n = s.recv_into(&mut buf, 0.5)?.n_samples;
        if n < buf.len() {
            continue;
        }
        let x = &buf[..n];
        tone += power(x, TONE_OFFSET_HZ, rate);
        for (k, &o) in spur_band.iter().enumerate() {
            spur[k] += power(x, TONE_OFFSET_HZ + o, rate);
        }
        floor += floor_band
            .iter()
            .map(|&o| power(x, TONE_OFFSET_HZ + o, rate))
            .sum::<f64>()
            / floor_band.len() as f64;
        used += 1;
    }
    let lost = s.finish()?.report.lost;
    let (k, &worst) = spur
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(b.1))
        .expect("a band");
    Ok(format!(
        "{{\"rate\":{rate},\"master_clock\":{mcr},\"decimation\":{},\"tone_db\":{:.1},\"worst_in_spur_band_dbc\":{:.1},\"at_offset_hz\":{},\"floor_dbc\":{:.1},\"worst_above_floor_db\":{:.1},\"receive_len\":{},\"lost\":\"{lost:?}\"}}",
        (mcr / rate).round(),
        db(tone / RECEIVES as f64),
        db(worst) - db(tone),
        spur_band[k],
        db(floor) - db(tone),
        db(worst) - db(floor),
        buf.len()
    ))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.first().map(String::as_str) != Some("--cabled") || args.len() < 3 {
        eprintln!("usage: spurcheck --cabled <hackrf spec> <usrp spec> [rate …]");
        std::process::exit(2);
    }
    let rates: Vec<f64> = args[3..].iter().filter_map(|s| s.parse().ok()).collect();
    let rates = if rates.is_empty() {
        vec![19.2e6, 4e6]
    } else {
        rates
    };

    // The tone: the HackRF's LO at TONE_OFFSET_HZ above the USRP's centre,
    // fed from a thread with its baseband at DC + a small offset.
    let tx_rate = 4e6;
    let mut h = Device::open(&args[1])?;
    h.set_rate(tx_rate)?;
    {
        let hk = h.as_hackrf().expect("a HackRF");
        sdr_core::block_on(hk.set_amp(false))?;
        sdr_core::block_on(hk.set_txvga_gain(HACKRF_TXVGA_DB))?;
        sdr_core::block_on(hk.set_freq(&TuneRequest::auto(RX_HZ + TONE_OFFSET_HZ - 100e3)))?;
    }
    let mut tx = h
        .into_tx_stream(&StreamConfig::default())
        .map_err(|e| e.error)?;
    let keyed = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let feeding = std::sync::Arc::clone(&keyed);
    let feeder = thread::spawn(
        move || -> sdr_driver::Result<sdr_driver::blocking::TxStream> {
            let a = 32767.0 * 0.5;
            let step = TAU * 100e3 / tx_rate;
            let mut ph = 0.0f64;
            let mut chunk = vec![Iq::default(); tx.samples_per_transfer()];
            while feeding.load(std::sync::atomic::Ordering::Relaxed) {
                for s in chunk.iter_mut() {
                    *s = Iq {
                        i: (a * ph.cos()) as i16,
                        q: (a * ph.sin()) as i16,
                    };
                    ph = (ph + step) % TAU;
                }
                tx.send(&chunk)?;
            }
            Ok(tx)
        },
    );
    thread::sleep(Duration::from_millis(500));
    for &rate in &rates {
        for decim1 in [true, false] {
            match measure(&args[2], rate, decim1) {
                Ok(line) => println!("{line}"),
                Err(e) => {
                    println!("{{\"rate\":{rate},\"decimation_1\":{decim1},\"error\":\"{e}\"}}")
                }
            }
        }
    }
    keyed.store(false, std::sync::atomic::Ordering::Relaxed);
    feeder.join().expect("the feeding thread")?.finish()?;
    Ok(())
}
