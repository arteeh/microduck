//! Beta FIFO delivery from protocol-2 physics. The body runs a documented game-
//! rotation approximation; robotd applies the exact real-reader chip-to-head transform.
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;

use crate::head_imu::{self, HeadImuStatus};

#[derive(serde::Deserialize)]
struct Batch {
    samples: Vec<Frame>,
}
#[derive(serde::Deserialize)]
struct Frame {
    seq: u64,
    t_ns: u64,
    gyro: [f32; 3],
    accel: [f32; 3],
    quat: [f32; 4],
    temp_c: f32,
}

pub fn run(
    addr: &str,
    status: &HeadImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &AtomicBool,
) {
    let started = Instant::now();
    let mut seq = 0;
    while !shutdown.load(Ordering::Acquire) {
        if let Err(error) = stream(addr, status, frames, shutdown, started, &mut seq) {
            status.lost(format!("simulated beta head IMU: {error}"));
            tracing::warn!(%error, "simulated beta head IMU unavailable; retrying");
            sleep(Duration::from_millis(500), shutdown);
        }
    }
}

fn stream(
    addr: &str,
    status: &HeadImuStatus,
    frames: &tokio::sync::broadcast::Sender<proto::HeadImuFrame>,
    shutdown: &AtomicBool,
    started: Instant,
    seq: &mut u64,
) -> Result<(), Box<dyn std::error::Error>> {
    let address = addr.to_socket_addrs()?.next().ok_or("no body address")?;
    let socket = TcpStream::connect_timeout(&address, Duration::from_millis(500))?;
    socket.set_nodelay(true)?;
    socket.set_read_timeout(Some(Duration::from_millis(500)))?;
    socket.set_write_timeout(Some(Duration::from_millis(500)))?;
    let mut write = socket.try_clone()?;
    let mut read = BufReader::new(socket);
    let before = proto::clock::monotonic_ns();
    writeln!(write, "{{\"op\":\"hello\",\"protocol\":2,\"joints\":15}}")?;
    let mut line = String::new();
    read.read_line(&mut line)?;
    let after = proto::clock::monotonic_ns();
    let hello: serde_json::Value = serde_json::from_str(&line)?;
    if hello["protocol"] != 2 || hello["board"] != "beta" {
        return Err(format!("expected body protocol 2, board beta; received {hello}; update microduck_rl and start --board beta").into());
    }
    let clock = hello["clock_ns"]
        .as_u64()
        .ok_or("body handshake lacks clock_ns")?;
    let offset = i128::from(before / 2 + after / 2) - i128::from(clock);
    let mut cursor = 0;
    status.found("LSM6DSV16X");
    while !shutdown.load(Ordering::Acquire) {
        let tick = Instant::now();
        writeln!(write, "{{\"op\":\"head_imu\",\"after\":{cursor}}}")?;
        line.clear();
        read.read_line(&mut line)?;
        let batch: Batch = serde_json::from_str(&line)?;
        // Same bounded FIFO drain as the real reader: 48 words / 3 words per sample.
        let skip = batch.samples.len().saturating_sub(16);
        for raw in batch.samples.into_iter().skip(skip) {
            cursor = raw.seq;
            let t_ns = (i128::from(raw.t_ns) + offset).clamp(0, i128::from(u64::MAX)) as u64;
            if proto::clock::monotonic_ns().saturating_sub(t_ns) > 300_000_000 {
                continue;
            }
            if !raw
                .gyro
                .iter()
                .chain(raw.accel.iter())
                .chain(raw.quat.iter())
                .all(|v| v.is_finite())
                || !raw.temp_c.is_finite()
            {
                return Err("nonfinite head IMU sample".into());
            }
            let sample = head_imu::to_head(raw.gyro, raw.accel, raw.quat);
            *seq += 1;
            let _ = frames.send(proto::HeadImuFrame {
                seq: *seq,
                at_us: started.elapsed().as_micros() as u64,
                t_ns,
                gyro: sample.gyro,
                accel: sample.accel,
                quat: sample.quat,
                temp_c: raw.temp_c,
            });
        }
        sleep(
            Duration::from_secs_f64(1.0 / f64::from(head_imu::WAKE_HZ))
                .saturating_sub(tick.elapsed()),
            shutdown,
        );
    }
    Ok(())
}

fn sleep(duration: Duration, shutdown: &AtomicBool) {
    let deadline = Instant::now() + duration;
    while !shutdown.load(Ordering::Acquire) && Instant::now() < deadline {
        std::thread::sleep(
            deadline
                .saturating_duration_since(Instant::now())
                .min(Duration::from_millis(20)),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::sync::Arc;

    #[test]
    fn beta_fifo_burst_preserves_capture_times_and_real_reader_axes() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap().to_string();
        let shutdown = Arc::new(AtomicBool::new(false));
        let stop = shutdown.clone();
        let body = std::thread::spawn(move || {
            let (mut socket, _) = listener.accept().unwrap();
            let mut read = BufReader::new(socket.try_clone().unwrap());
            let mut request = String::new();
            read.read_line(&mut request).unwrap();
            let clock = proto::clock::monotonic_ns();
            writeln!(
                socket,
                "{{\"protocol\":2,\"board\":\"beta\",\"clock_ns\":{clock}}}"
            )
            .unwrap();
            request.clear();
            read.read_line(&mut request).unwrap();
            let samples: Vec<_> = (0..4)
                .map(|i| {
                    serde_json::json!({
                        "seq": i+1, "t_ns": clock-50_000_000+i*16_666_667,
                        "gyro": [0,1,0], "accel": [0,9.80665,0],
                        "quat": head_imu::MOUNT, "temp_c": 32
                    })
                })
                .collect();
            writeln!(socket, "{}", serde_json::json!({"samples": samples})).unwrap();
            stop.store(true, Ordering::Release);
        });
        let (frames, mut received) = tokio::sync::broadcast::channel(16);
        stream(
            &addr,
            &HeadImuStatus::new(),
            &frames,
            &shutdown,
            Instant::now(),
            &mut 0,
        )
        .unwrap();
        body.join().unwrap();
        let mut samples = Vec::new();
        while let Ok(frame) = received.try_recv() {
            samples.push(frame);
        }
        assert_eq!(samples.len(), 4);
        assert_eq!(samples[1].t_ns - samples[0].t_ns, 16_666_667);
        assert!((samples[0].gyro[2] - 1.0).abs() < 1e-6);
        assert!((samples[0].accel[2] - 9.80665).abs() < 1e-5);
        assert!((samples[0].quat[0] - 1.0).abs() < 1e-6);
    }
}
