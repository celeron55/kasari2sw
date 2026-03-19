use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use ringbuffer::RingBuffer;

// Task that drains the log ring buffer to USB CDC
#[embassy_executor::task]
pub async fn usb_cdc_log_drain_task(driver: Driver<'static, USB_OTG_FS>) -> ! {
    use embassy_time::Timer;

    // Create embassy-usb Config
    let mut config = Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("Kasari");
    config.product = Some("Kasari2sw STM32F722");
    config.serial_number = Some("123456");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Device descriptor fields
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create CDC-ACM class
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the USB device
    let mut usb = builder.build();

    // Run the USB device in the background
    let usb_fut = usb.run();

    // Create log drain future
    let log_drain_fut = async {
        loop {
            // Wait for USB to be connected and class to be ready
            class.wait_connection().await;

            // Now drain logs to USB CDC
            loop {
                // Collect pending log data
                let mut pending: heapless::Vec<u8, 256> = heapless::Vec::new();

                cortex_m::interrupt::free(|cs| {
                    let mut ring = crate::logging::get_usb_ring_buffer().borrow(cs).borrow_mut();
                    while pending.len() < pending.capacity() {
                        if let Some(byte) = ring.dequeue() {
                            let _ = pending.push(byte);
                        } else {
                            break;
                        }
                    }
                });

                // Send to USB CDC if we have data
                if !pending.is_empty() {
                    match class.write_packet(&pending).await {
                        Ok(_) => {}
                        Err(EndpointError::Disabled) => {
                            // USB disconnected, break inner loop and wait for reconnection
                            break;
                        }
                        Err(_) => {
                            // Other error, just continue
                        }
                    }
                }

                Timer::after_millis(10).await;
            }
        }
    };

    // Run both USB device and log drain concurrently
    embassy_futures::join::join(usb_fut, log_drain_fut).await;

    // This should never return
    loop {
        Timer::after_millis(1000).await;
    }
}
