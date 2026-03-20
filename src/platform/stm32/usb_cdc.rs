use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use ringbuffer::RingBuffer;

use crate::console::ConsoleMutex;

/// USB CDC console task - handles both TX drain and RX command processing
#[embassy_executor::task]
pub async fn usb_cdc_console_task(
    driver: Driver<'static, USB_OTG_FS>,
    console: &'static ConsoleMutex,
) -> ! {
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

    // Create console I/O future
    let console_fut = async {
        loop {
            // Wait for USB to be connected and class to be ready
            class.wait_connection().await;

            // Connection established - handle I/O
            loop {
                // 1. Try to read incoming data (commands) - non-blocking with short timeout
                let mut rx_buf = [0u8; 64];

                // Use select to check for RX data without blocking the TX drain
                use embassy_futures::select::{select, Either};
                use embassy_time::Duration;

                let read_fut = class.read_packet(&mut rx_buf);
                let timeout_fut = Timer::after(Duration::from_millis(1));

                match select(read_fut, timeout_fut).await {
                    Either::First(Ok(n)) if n > 0 => {
                        // Process received bytes
                        cortex_m::interrupt::free(|cs| {
                            let mut state = console.borrow(cs).borrow_mut();
                            for &byte in &rx_buf[..n] {
                                if let Some(cmd) = state.process_rx_byte(byte) {
                                    let response = state.execute_command(&cmd);
                                    state.write_bytes(response.as_bytes());
                                }
                            }
                        });
                    }
                    Either::First(Err(EndpointError::Disabled)) => {
                        // USB disconnected
                        break;
                    }
                    _ => {
                        // Timeout or other error - continue with TX drain
                    }
                }

                // 2. Drain output ring buffer to TX
                let mut pending: heapless::Vec<u8, 256> = heapless::Vec::new();

                cortex_m::interrupt::free(|cs| {
                    let mut state = console.borrow(cs).borrow_mut();
                    while pending.len() < pending.capacity() {
                        if let Some(byte) = state.output_ring.dequeue() {
                            let _ = pending.push(byte);
                        } else {
                            break;
                        }
                    }
                });

                if !pending.is_empty() {
                    match class.write_packet(&pending).await {
                        Ok(_) => {}
                        Err(EndpointError::Disabled) => {
                            // USB disconnected
                            break;
                        }
                        Err(_) => {
                            // Other error, continue
                        }
                    }
                }

                Timer::after_millis(10).await;
            }
        }
    };

    // Run both USB device and console I/O concurrently
    embassy_futures::join::join(usb_fut, console_fut).await;

    // This should never return
    loop {
        Timer::after_millis(1000).await;
    }
}
