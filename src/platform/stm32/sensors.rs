use embassy_stm32::usart::RingBufferedUartRx;
use embassy_sync::pubsub::Publisher;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_io_async::Read;
use log::info;

use kasarisw::shared::kasari::InputEvent;

// TFA300 LIDAR Protocol Constants
pub const PACKET_SIZE: usize = 9; // 9 bytes for TFA300 (was 22 for LDS02RR)
pub const HEAD_BYTE_1: u8 = 0x59; // First header byte
pub const HEAD_BYTE_2: u8 = 0x59; // Second header byte
pub const LIDAR_DISTANCE_OFFSET: f32 = 36.0; // Distance offset based on mounting position (may need retuning)

// Note: No LIDAR_ENCODER_HZ needed! TFA300 doesn't require encoder signal (major simplification)

pub struct ParsedPacket {
    pub distance: u16, // mm (TFA300 gives 1 value per packet)
}

/// Parse TFA300 9-byte UART packet
/// Format: [0x59, 0x59, DIST_L, DIST_H, STRENGTH_L, STRENGTH_H, 0x00, 0x00, CHECKSUM]
pub fn parse_packet(packet: &[u8]) -> Option<ParsedPacket> {
    if packet.len() != PACKET_SIZE || packet[0] != HEAD_BYTE_1 || packet[1] != HEAD_BYTE_2 {
        return None;
    }

    // Distance: little-endian u16 (mm)
    let distance = (packet[3] as u16) << 8 | (packet[2] as u16);

    // Signal strength: little-endian u16
    let _strength = (packet[5] as u16) << 8 | (packet[4] as u16);

    // Checksum: sum of bytes 0-7, masked to lower 8 bits
    let checksum = (packet[0..8].iter().map(|&b| b as u16).sum::<u16>() & 0xFF) as u8;
    if checksum != packet[8] {
        return None;
    }

    Some(ParsedPacket { distance })
}

/// Compute TFA300 checksum (simple sum of bytes, lower 8 bits)
pub fn compute_checksum(data: &[u8]) -> u8 {
    (data.iter().map(|&b| b as u16).sum::<u16>() & 0xFF) as u8
}

/// LIDAR publisher task - reads TFA300 packets from UART2 and publishes Lidar events
#[embassy_executor::task]
pub async fn lidar_publisher(
    mut uart_rx: RingBufferedUartRx<'static>,
    publisher: Publisher<'static, CriticalSectionRawMutex, InputEvent, 64, 3, 6>,
) {
    info!("LIDAR task started");

    let mut buffer = [0u8; PACKET_SIZE * 16]; // Buffer for multiple packets
    let mut pos = 0usize;
    let mut samples = [0.0f32; 10];
    let mut sample_idx = 0usize;
    let mut packet_count = 0u32;
    let mut error_count = 0u32;

    loop {
        // Read available data into buffer
        match uart_rx.read(&mut buffer[pos..]).await {
            Ok(n) if n > 0 => {
                pos += n;

                // Process complete packets
                while pos >= PACKET_SIZE {
                    // Look for header bytes
                    if buffer[0] == HEAD_BYTE_1 && buffer[1] == HEAD_BYTE_2 {
                        if let Some(parsed) = parse_packet(&buffer[0..PACKET_SIZE]) {
                            // Apply distance offset
                            let d = parsed.distance as f32 + LIDAR_DISTANCE_OFFSET;
                            samples[sample_idx] = d.max(0.0);
                            sample_idx += 1;
                            packet_count += 1;

                            // Publish batch of 10 samples
                            if sample_idx >= 10 {
                                let ts = embassy_time::Instant::now().as_micros() as u64;
                                let event = InputEvent::Lidar(ts, samples);
                                publisher.publish_immediate(event);
                                sample_idx = 0;
                            }
                        } else {
                            error_count += 1;
                        }
                        // Shift buffer past this packet
                        buffer.copy_within(PACKET_SIZE..pos, 0);
                        pos -= PACKET_SIZE;
                    } else {
                        // Bad header, skip one byte to resync
                        buffer.copy_within(1..pos, 0);
                        pos -= 1;
                        error_count += 1;
                    }
                }

                // Log stats periodically
                if packet_count > 0 && packet_count % 10000 == 0 {
                    info!("LIDAR: {} packets, {} errors", packet_count, error_count);
                }
            }
            Ok(_) => {
                // Zero bytes read, continue
            }
            Err(_e) => {
                // UART error, reset buffer
                pos = 0;
                error_count += 1;
            }
        }
    }
}

// ============================================================================
// TODO: SENSOR TASKS (awaiting Mamba F722APP pinout)
// ============================================================================

// TODO: LIDAR Publisher Task
// Reads TFA300 packets from UART RX (with DMA) and publishes Lidar events
//
// #[embassy_executor::task]
// pub async fn lidar_publisher(
//     mut uart_rx: embassy_stm32::usart::UartRx<'static>,
//     event_channel: &'static crate::shared::EventChannel,
// ) {
//     use crate::shared::kasari::InputEvent;
//
//     let publisher = event_channel.publisher().unwrap();
//     let mut buffer = [0u8; PACKET_SIZE * 8];  // Buffer for multiple packets
//     let mut pos = 0;
//     let mut samples = [0.0f32; 10];
//     let mut sample_idx = 0;
//
//     loop {
//         // Read data asynchronously with DMA
//         match uart_rx.read(&mut buffer[pos..]).await {
//             Ok(n) => {
//                 pos += n;
//
//                 // Parse complete packets
//                 while pos >= PACKET_SIZE {
//                     // Look for header bytes
//                     if buffer[0] == HEAD_BYTE_1 && buffer[1] == HEAD_BYTE_2 {
//                         if let Some(parsed) = parse_packet(&buffer[0..PACKET_SIZE]) {
//                             // Process distance with offset
//                             let d = parsed.distance as f32 + LIDAR_DISTANCE_OFFSET;
//                             samples[sample_idx] = d.max(0.0);
//                             sample_idx += 1;
//
//                             if sample_idx >= 10 {
//                                 let ts = embassy_time::Instant::now().as_micros() as u64;
//                                 let event = InputEvent::Lidar(ts, samples);
//                                 publisher.publish_immediate(event);
//                                 sample_idx = 0;
//                             }
//                         }
//                         // Shift buffer
//                         buffer.copy_within(PACKET_SIZE.., 0);
//                         pos -= PACKET_SIZE;
//                     } else {
//                         // Bad header, skip one byte
//                         buffer.copy_within(1.., 0);
//                         pos -= 1;
//                     }
//                 }
//             }
//             Err(_) => {
//                 // UART error, reset
//                 pos = 0;
//             }
//         }
//     }
// }

// TODO: Accelerometer Task
// Reads ADXL373 via SPI and publishes Accelerometer events
// Protocol is same as ESP32 version, just different HAL
//
// #[embassy_executor::task]
// pub async fn accelerometer_task(
//     mut spi: embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Blocking>,
//     mut cs: embassy_stm32::gpio::Output<'static>,
//     event_channel: &'static crate::shared::EventChannel,
// ) {
//     use crate::shared::kasari::InputEvent;
//     use embassy_time::{Timer, Duration};
//
//     let publisher = event_channel.publisher().unwrap();
//
//     cs.set_high();
//     Timer::after_millis(10).await;
//
//     // Read device ID to verify connection (should be 0xAD)
//     cs.set_low();
//     let mut buf = [(0x00 << 1) | 1, 0];  // DEVID_AD register
//     spi.blocking_transfer_in_place(&mut buf).unwrap();
//     cs.set_high();
//     defmt::info!("ADXL373 DEVID_AD = {:#02x} (should be 0xAD)", buf[1]);
//
//     // Configure power control: measurement mode, full bandwidth
//     cs.set_low();
//     let mut buf = [(0x2D << 1) | 0, 0b00000010];  // POWER_CTL register
//     spi.blocking_transfer_in_place(&mut buf).unwrap();
//     cs.set_high();
//     Timer::after_millis(10).await;
//
//     loop {
//         // Read X, Y, Z data (12-bit signed, 0.2G per LSB)
//         cs.set_low();
//         let mut buf = [(0x08 << 1) | 1, 0, 0, 0, 0, 0, 0];  // XDATA register
//         spi.blocking_transfer_in_place(&mut buf).unwrap();
//         cs.set_high();
//
//         // Parse 12-bit signed values
//         let x_raw = (((buf[1] as i16) << 8) | buf[2] as i16) >> 4;
//         let y_raw = (((buf[3] as i16) << 8) | buf[4] as i16) >> 4;
//         let z_raw = (((buf[5] as i16) << 8) | buf[6] as i16) >> 4;
//
//         // Convert to G (0.2G per LSB)
//         let x_g = x_raw as f32 * 0.2;
//         let y_g = y_raw as f32 * 0.2;
//         let z_g = z_raw as f32 * 0.2;
//
//         // Publish event (note: axes may need remapping based on mounting)
//         let timestamp = embassy_time::Instant::now().as_micros() as u64;
//         let event = InputEvent::Accelerometer(timestamp, -y_g, -z_g);
//         publisher.publish_immediate(event);
//
//         // Sample at ~100 Hz
//         Timer::after_millis(10).await;
//     }
// }

// TODO: RC Receiver Task
// Measures PWM pulse width using timer input capture
// Publishes Receiver events for mode switching
//
// #[embassy_executor::task]
// pub async fn rc_receiver_task(
//     mut capture: embassy_stm32::timer::input_capture::InputCapture<'static>,
//     event_channel: &'static crate::shared::EventChannel,
// ) {
//     use crate::shared::kasari::InputEvent;
//     use embassy_stm32::timer::input_capture::Channel;
//     use embassy_time::Duration;
//
//     let publisher = event_channel.publisher().unwrap();
//
//     loop {
//         // Wait for rising edge
//         match embassy_time::with_timeout(
//             Duration::from_millis(100),
//             capture.wait_for_rising_edge(Channel::Ch1)
//         ).await {
//             Ok(Ok(())) => {
//                 // Capture start time
//                 let start = capture.get_capture(Channel::Ch1);
//
//                 // Wait for falling edge
//                 if let Ok(Ok(())) = embassy_time::with_timeout(
//                     Duration::from_millis(50),
//                     capture.wait_for_falling_edge(Channel::Ch1)
//                 ).await {
//                     // Capture end time
//                     let end = capture.get_capture(Channel::Ch1);
//
//                     // Calculate pulse width in microseconds
//                     let pulse_width_us = end.wrapping_sub(start) as f32;
//
//                     // Publish event
//                     let timestamp = embassy_time::Instant::now().as_micros() as u64;
//                     let event = InputEvent::Receiver(timestamp, 0, Some(pulse_width_us));
//                     publisher.publish_immediate(event);
//                 }
//             }
//             Err(_) => {
//                 // Timeout - no signal
//                 let timestamp = embassy_time::Instant::now().as_micros() as u64;
//                 let event = InputEvent::Receiver(timestamp, 0, None);
//                 publisher.publish_immediate(event);
//             }
//         }
//     }
// }

// TODO: Battery Voltage Monitor Task
// Reads ADC and publishes VBat events
//
// #[embassy_executor::task]
// pub async fn battery_monitor_task(
//     mut adc: embassy_stm32::adc::Adc<'static, embassy_stm32::mode::Blocking>,
//     vbat_pin: impl embassy_stm32::adc::AdcChannel<embassy_stm32::peripherals::ADC1>,
//     event_channel: &'static crate::shared::EventChannel,
// ) {
//     use crate::shared::kasari::InputEvent;
//     use embassy_time::{Timer, Duration};
//
//     let publisher = event_channel.publisher().unwrap();
//
//     loop {
//         // Read ADC value
//         let raw = adc.blocking_read(&mut vbat_pin);
//
//         // Convert to voltage (calibration needed for specific voltage divider)
//         // Original ESP32: (4095 - raw) * 0.01045
//         // STM32 ADC is 12-bit (0-4095), calibration will differ
//         let voltage = raw as f32 * 0.01;  // TBD: needs calibration
//
//         // Publish event
//         let timestamp = embassy_time::Instant::now().as_micros() as u64;
//         let event = InputEvent::VBat(timestamp, voltage);
//         publisher.publish_immediate(event);
//
//         // Sample at 10 Hz
//         Timer::after_millis(100).await;
//     }
// }
