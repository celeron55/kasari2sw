use embassy_rp::uart::{UartRx, Async};
use embassy_rp::peripherals::UART1;
use embassy_executor::task;
use embassy_time::Timer;

#[task]
pub async fn lidar_task(
    mut rx: UartRx<'static, Async>,
) {
    let mut packet_buf = [0u8; 9];

    loop {
        match rx.read(&mut packet_buf).await {
            Ok(_) => {
                if packet_buf[0] == 0x59 && packet_buf[1] == 0x59 {
                    let dist_mm = ((packet_buf[3] as u16) << 8 | packet_buf[2] as u16) as f32;
                    let timestamp = embassy_time::Instant::now().as_micros();
                    defmt::info!("LIDAR: {}mm", dist_mm);
                    // TODO: publish to EventChannel
                }
            }
            Err(e) => {
                defmt::warn!("LIDAR read error: {:?}", e);
                Timer::after_millis(1).await;
            }
        }
    }
}
