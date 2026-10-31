//! sdr-driver on the bench, through its common API only.
//!
//!   cargo run --release -p sdr-driver --features uhd --example drivercheck -- [--cabled]
//!
//! 1. Every attached radio receives for 2 s at 2 Msps while a second thread
//!    retunes it three times; the stream is finished and its report printed
//!    (samples, loss with its certainty, the breaks).
//! 2. With `--cabled` — the operator's statement that the transmitters'
//!    ports are cabled to the receivers through fixed attenuation and
//!    nothing radiates — every radio that can transmit sends a tone
//!    ([`TxStream`]) while every other attached radio receives it
//!    ([`Stream`]); the tone's power is read against its neighbourhood.
//!    Transmit gain stays low: a HackRF's at 0 dB with the amp off, a
//!    USRP's at `USRP_TX_GAIN_DB`.
//!
//! One JSON line per step.

use std::f64::consts::TAU;
use std::thread;
use std::time::{Duration, Instant};

use sdr_driver::blocking::{Device, Stream};
use sdr_driver::{Backend, Iq, StreamConfig, TuneRequest};

const RATE: f64 = 2e6;
const RX_HZ: f64 = 300e6;
const RX_GAIN_DB: f64 = 30.0;
const RETUNES: usize = 3;
const RECEIVE_FOR: Duration = Duration::from_secs(2);
/// The tone: this far above the receiver's centre, this far below full
/// scale.
const TONE_OFFSET_HZ: f64 = 300e3;
const BACKOFF_DB: f64 = -20.0;
const USRP_TX_GAIN_DB: f64 = 60.0;
/// Frequencies the tone's power is compared against, Hz from centre.
const NEIGHBOURS_HZ: [f64; 6] = [-500e3, -200e3, 100e3, 150e3, 450e3, 600e3];

fn open(spec: &str) -> Result<Device, Box<dyn std::error::Error>> {
    let mut d = Device::open(spec)?;
    d.set_rate(RATE)?;
    d.set_freq(&TuneRequest::auto(RX_HZ))?;
    d.set_gain(RX_GAIN_DB)?;
    Ok(d)
}

/// |Σ x·e^{-j2πfn/fs}|² / N², the power at `f`.
fn power(x: &[Iq], f: f64) -> f64 {
    let (mut re, mut im) = (0.0, 0.0);
    for (n, s) in x.iter().enumerate() {
        let ph = -TAU * f * n as f64 / RATE;
        let (c, sn) = (ph.cos(), ph.sin());
        re += s.i as f64 * c - s.q as f64 * sn;
        im += s.i as f64 * sn + s.q as f64 * c;
    }
    (re * re + im * im) / (x.len() as f64).powi(2)
}

/// Receive for `RECEIVE_FOR` with retunes from a second thread.
fn receive_with_retunes(spec: &str) -> Result<(), Box<dyn std::error::Error>> {
    let d = open(spec)?;
    let label = d.label();
    let (mut reader, control) = d
        .into_stream(&StreamConfig::default())
        .map_err(|e| e.error)?
        .split();
    let retuner = thread::spawn(move || {
        let mut breaks = Vec::new();
        for k in 0..RETUNES {
            thread::sleep(RECEIVE_FOR / (RETUNES as u32 + 1));
            let hz = RX_HZ + 100e3 * ((k + 1) % 2) as f64;
            breaks.push(control.retune(&TuneRequest::auto(hz)).map(|(_, b)| b));
        }
        (control, breaks)
    });
    let mut buf = vec![Iq::default(); reader.max_num_samps()];
    let t0 = Instant::now();
    let mut received = 0u64;
    while t0.elapsed() < RECEIVE_FOR {
        received += reader.recv_into(&mut buf, 0.5)?.n_samples as u64;
    }
    let (control, breaks) = retuner.join().expect("the retuning thread");
    let f = Stream::reunite(reader, control)
        .map_err(|_| "reunite")?
        .finish()?;
    let r = &f.report;
    println!(
        "{{\"step\":\"receive\",\"radio\":\"{label}\",\"spec\":\"{spec}\",\"received\":{received},\"samples\":{},\"tail\":{},\"lost\":\"{:?}\",\"breaks\":{:?},\"retune_errors\":{}}}",
        r.samples,
        f.tail.len(),
        r.lost,
        r.breaks
            .iter()
            .map(|b| (b.at_sample, b.settled_at))
            .collect::<Vec<_>>(),
        breaks.iter().filter(|b| b.is_err()).count()
    );
    Ok(())
}

/// `tx_spec` sends a tone; `rx_spec` receives it.
fn tone(tx_spec: &str, rx_spec: &str) -> Result<(), Box<dyn std::error::Error>> {
    let mut tx_dev = Device::open(tx_spec)?;
    let tx_label = tx_dev.label();
    tx_dev.set_rate(RATE)?;
    match tx_dev.caps().backend {
        Backend::HackRf => {
            let h = tx_dev.as_hackrf().expect("a HackRF");
            sdr_core::block_on(h.set_amp(false))?;
            sdr_core::block_on(h.set_txvga_gain(0))?;
            sdr_core::block_on(h.set_freq(&TuneRequest::auto(RX_HZ)))?;
        }
        #[cfg(feature = "uhd")]
        Backend::Usrp => {
            let u = tx_dev.as_uhd().expect("a USRP");
            u.set_tx_rate(RATE, 0)?;
            u.set_tx_antenna("TX/RX", 0)?;
            u.set_tx_gain(USRP_TX_GAIN_DB, 0)?;
            u.set_tx_freq(&TuneRequest::auto(RX_HZ), 0)?;
        }
        other => return Err(format!("{other:?} does not transmit").into()),
    }
    let mut tx = tx_dev
        .into_tx_stream(&StreamConfig::default())
        .map_err(|e| e.error)?;
    let mut rx_dev = open(rx_spec)?;
    let rx_label = rx_dev.label();
    // A B2xx receives on RX2 by default; the cable is on TX/RX.
    #[cfg(feature = "uhd")]
    if let Some(u) = rx_dev.as_uhd() {
        u.set_rx_antenna("TX/RX", 0)?;
    }
    let mut rx = rx_dev
        .into_stream(&StreamConfig::default())
        .map_err(|e| e.error)?;

    // The tone at +TONE_OFFSET_HZ, fed from this thread while the receiver
    // reads on another.
    let amplitude = 32767.0 * 10f64.powf(BACKOFF_DB / 20.0);
    let step = TAU * TONE_OFFSET_HZ / RATE;
    let mut phase = 0.0f64;
    let mut chunk = vec![Iq::default(); tx.samples_per_transfer()];
    let mut fill = |chunk: &mut [Iq]| {
        for s in chunk.iter_mut() {
            *s = Iq {
                i: (amplitude * phase.cos()) as i16,
                q: (amplitude * phase.sin()) as i16,
            };
            phase = (phase + step) % TAU;
        }
    };
    let reader = thread::spawn(move || -> sdr_driver::Result<(Vec<Iq>, Stream)> {
        // Let the transmitter key and settle, then keep 100 ms.
        thread::sleep(Duration::from_millis(500));
        let mut buf = vec![Iq::default(); rx.max_num_samps()];
        let mut kept = Vec::new();
        let want = (RATE * 0.1) as usize;
        while kept.len() < want {
            let n = rx.recv_into(&mut buf, 0.5)?.n_samples;
            kept.extend_from_slice(&buf[..n]);
        }
        Ok((kept, rx))
    });
    let t0 = Instant::now();
    while !reader.is_finished() && t0.elapsed() < Duration::from_secs(5) {
        fill(&mut chunk);
        tx.send(&chunk)?;
    }
    let (kept, rx) = reader.join().expect("the reading thread")?;
    let tx_report = tx.finish()?.report;
    let rx_report = rx.finish()?.report;
    let kept = &kept[kept.len().saturating_sub(65536)..];
    let tone_db = 10.0 * power(kept, TONE_OFFSET_HZ).log10();
    let floor_db = 10.0
        * (NEIGHBOURS_HZ.iter().map(|&f| power(kept, f)).sum::<f64>() / NEIGHBOURS_HZ.len() as f64)
            .log10();
    println!(
        "{{\"step\":\"tone\",\"from\":\"{tx_label}\",\"to\":\"{rx_label}\",\"tone_db\":{tone_db:.1},\"neighbours_db\":{floor_db:.1},\"above_db\":{:.1},\"tx_samples\":{},\"tx_underruns\":{:?},\"rx_lost\":\"{:?}\"}}",
        tone_db - floor_db,
        tx_report.samples,
        tx_report.underruns,
        rx_report.lost
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cabled = std::env::args().any(|a| a == "--cabled");
    let radios = sdr_driver::blocking::list();
    for r in &radios {
        println!(
            "{{\"step\":\"list\",\"spec\":\"{}\",\"label\":\"{}\",\"backend\":\"{:?}\"}}",
            r.spec, r.label, r.backend
        );
    }
    for r in &radios {
        if let Err(e) = receive_with_retunes(&r.spec) {
            println!(
                "{{\"step\":\"receive\",\"spec\":\"{}\",\"error\":\"{e}\"}}",
                r.spec
            );
        }
    }
    if cabled {
        for tx in radios.iter().filter(|r| r.backend != Backend::Rtl) {
            for rx in radios.iter().filter(|r| r.spec != tx.spec) {
                if let Err(e) = tone(&tx.spec, &rx.spec) {
                    println!(
                        "{{\"step\":\"tone\",\"from\":\"{}\",\"to\":\"{}\",\"error\":\"{e}\"}}",
                        tx.spec, rx.spec
                    );
                }
            }
        }
    }
    Ok(())
}
