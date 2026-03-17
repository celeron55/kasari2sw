use embassy_rp::uart::BufferedUartRx;
use embassy_executor::task;
use embassy_time::Timer;
use embedded_io_async::Read;
use log::info;
use log::warn;

#[task]
pub async fn lidar_task(
    mut rx: BufferedUartRx,
) {
    let mut packet_buf = [0u8; 9];
    let mut samples = [0.0f32; 10];
    let mut sample_idx = 0;

    loop {
        match rx.read(&mut packet_buf).await {
            Ok(_) => {
                if packet_buf[0] == 0x59 && packet_buf[1] == 0x59 {
                    let dist_mm = ((packet_buf[3] as u16) << 8 | packet_buf[2] as u16) as f32;
                    samples[sample_idx] = dist_mm;
                    sample_idx += 1;

                    if sample_idx >= 10 {
                        let timestamp = embassy_time::Instant::now().as_micros();
                        info!("LIDAR: {:?}", samples);
                        // TODO: publish InputEvent::Lidar(timestamp, samples) to EventChannel
                        sample_idx = 0;
                    }
                }
            }
            Err(e) => {
                warn!("LIDAR read error: {:?}", e);
                Timer::after_millis(1).await;
            }
        }
    }
}
