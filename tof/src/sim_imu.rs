//! Raw chip-axis samples from physics, fused with bmi088-rs's Madgwick algorithm.
//! The body retains physics ticks: polling jitter may batch samples, never integrate a
//! head turn using the daemon wakeup interval or substitute ground-truth orientation.
use std::io::{BufRead, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use ahrs::{Ahrs, Madgwick};
use duck_ipc_proto as proto;
use nalgebra::Vector3;

use crate::imu::ImuStatus;

#[derive(serde::Deserialize)]
struct Samples {
    samples: Vec<RawSample>,
}

#[derive(serde::Deserialize)]
struct RawSample {
    seq: u64,
    t_ns: u64,
    gyro: [f32; 3],
    accel: [f32; 3],
    temp_c: f32,
}

pub fn run(
    addr: &str,
    hz: u8,
    status: &ImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &Arc<AtomicBool>,
) {
    let period = Duration::from_secs_f64(1.0 / f64::from(hz.max(1)));
    let started = Instant::now();
    let mut seq = 0;
    while !shutdown.load(Ordering::Acquire) {
        let result = stream(addr, period, started, &mut seq, status, frames, shutdown);
        if let Err(error) = result {
            tracing::warn!(%addr, %error, "simulated head IMU unavailable; retrying");
            status.lost(format!("simulated head IMU: {error}"));
            crate::sleep_unless_shutdown(Duration::from_secs(1), shutdown);
        }
    }
}

fn stream(
    addr: &str,
    period: Duration,
    started: Instant,
    seq: &mut u64,
    status: &ImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &Arc<AtomicBool>,
) -> anyhow::Result<()> {
    let (mut write, mut read, offset) = crate::connect_sim(addr)?;
    let mut after = 0;
    let mut last_ns = None;
    let mut filter = Madgwick::<f64>::new(period.as_secs_f64(), 0.1);
    status.found("BMI088");
    while !shutdown.load(Ordering::Acquire) {
        writeln!(write, "{{\"op\":\"head_imu\",\"after\":{after}}}")?;
        let mut line = String::new();
        read.read_line(&mut line)?;
        let batch: Samples = serde_json::from_str(&line)?;
        for raw in batch.samples {
            after = raw.seq;
            anyhow::ensure!(
                raw.gyro
                    .iter()
                    .chain(raw.accel.iter())
                    .all(|v| v.is_finite())
                    && raw.temp_c.is_finite(),
                "nonfinite IMU sample"
            );
            let dt = last_ns.map_or(period.as_secs_f64(), |last| {
                raw.t_ns.saturating_sub(last) as f64 / 1e9
            });
            if dt + 1e-9 < period.as_secs_f64() {
                continue;
            }
            last_ns = Some(raw.t_ns);
            // Exactly bmi088-rs v0.1.2: recreate dt, retain quaternion, normalize
            // accelerometer inside update_imu, retain the filter on a zero gradient.
            filter = Madgwick::new_with_quat(dt.clamp(1e-4, 0.2), 0.1, filter.quat);
            let _ = filter.update_imu(
                &Vector3::from(raw.gyro.map(f64::from)),
                &Vector3::from(raw.accel.map(f64::from)),
            );
            let q = filter.quat.into_inner();
            *seq += 1;
            let _ = frames.send(proto::HeadImuFrame {
                seq: *seq,
                at_us: started.elapsed().as_micros() as u64,
                t_ns: (i128::from(raw.t_ns) + offset).clamp(0, i128::from(u64::MAX)) as u64,
                gyro: raw.gyro,
                accel: raw.accel,
                quat: [q.w as f32, q.i as f32, q.j as f32, q.k as f32],
                temp_c: raw.temp_c,
            });
        }
        crate::sleep_unless_shutdown(period, shutdown);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    #[test]
    fn old_body_version_is_a_named_error() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let body = std::thread::spawn(move || {
            let (mut socket, _) = listener.accept().unwrap();
            let mut request = String::new();
            std::io::BufReader::new(socket.try_clone().unwrap())
                .read_line(&mut request)
                .unwrap();
            socket.write_all(b"{\"protocol\":1}\n").unwrap();
        });
        let error = crate::connect_sim(&addr).unwrap_err().to_string();
        assert!(error.contains("expected 2"), "{error}");
        assert!(error.contains("update microduck_rl"), "{error}");
        body.join().unwrap();
    }

    #[test]
    fn batched_physics_uses_capture_intervals_and_publishes_at_100_hz() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let shutdown = Arc::new(AtomicBool::new(false));
        let stop = shutdown.clone();
        let body = std::thread::spawn(move || {
            let (mut socket, _) = listener.accept().unwrap();
            let mut read = std::io::BufReader::new(socket.try_clone().unwrap());
            let mut request = String::new();
            read.read_line(&mut request).unwrap();
            writeln!(
                socket,
                "{{\"protocol\":2,\"clock_ns\":{}}}",
                proto::clock::monotonic_ns()
            )
            .unwrap();
            request.clear();
            read.read_line(&mut request).unwrap();
            // 200 physics samples arrive in one batch. tofd reads at the chip's
            // configured 100 Hz, so integrate 100 captures, not 200 daemon polls.
            let samples: Vec<_> = (0..200)
                .map(|i| {
                    serde_json::json!({
                        "seq": i + 1, "t_ns": 1_000_000_000u64 + i * 5_000_000,
                        "gyro": [0.0, 0.0, 1.0], "accel": [0.0, 0.0, 9.80665], "temp_c": 32.0
                    })
                })
                .collect();
            writeln!(socket, "{}", serde_json::json!({"samples": samples})).unwrap();
            stop.store(true, Ordering::Release);
        });
        let (frames, mut received) = tokio::sync::broadcast::channel(256);
        let mut seq = 0;
        stream(
            &addr,
            Duration::from_millis(10),
            Instant::now(),
            &mut seq,
            &ImuStatus::new(100),
            &frames,
            &shutdown,
        )
        .unwrap();
        body.join().unwrap();
        let mut samples = Vec::new();
        while let Ok(frame) = received.try_recv() {
            samples.push(frame);
        }
        assert_eq!(samples.len(), 100);
        assert_eq!(samples[1].t_ns - samples[0].t_ns, 10_000_000);
        let q = samples.last().unwrap().quat;
        // One radian about Z, with no magnetometer to anchor yaw.
        assert!((q[0] - 0.5f32.cos()).abs() < 0.001, "{q:?}");
        assert!((q[3] - 0.5f32.sin()).abs() < 0.001, "{q:?}");
    }
}
