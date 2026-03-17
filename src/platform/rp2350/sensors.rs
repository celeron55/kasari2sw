use embassy_rp::uart::BufferedUartRx;
use embassy_executor::task;
use embassy_time::Timer;
use log::info;
use log::warn;
use embedded_io_async::Read;

#[task]
pub async fn lidar_task(
    mut rx: BufferedUartRx,
) {
    let mut packet_buf = [0u8; 9];

    loop {
        match rx.read(&mut packet_buf).await {
            Ok(_) => {
                if packet_buf[0] == 0x59 && packet_buf[1] == 0x59 {
                    let dist_mm = ((packet_buf[3] as u16) << 8 | packet_buf[2] as u16) as f32;
                    let _timestamp = embassy_time::Instant::now().as_micros();
                    info!("LIDAR: {}mm", dist_mm);
                    // TODO: publish to EventChannel
                }
            }
            Err(e) => {
                warn!("LIDAR read error: {:?}", e);
                Timer::after_millis(1).await;
            }
        }
    }
}
