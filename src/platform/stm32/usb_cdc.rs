use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use ringbuffer::RingBuffer;
use static_cell::StaticCell;
use log::info;

use crate::console::ConsoleMutex;
use crate::blheli_passthrough::{MspParser, FourWayInterface, init_motor_pins_gpio};

// MSP command IDs (from Betaflight msp_protocol.h)
const MSP_API_VERSION: u8 = 1;
const MSP_FC_VARIANT: u8 = 2;
const MSP_FC_VERSION: u8 = 3;
const MSP_BOARD_INFO: u8 = 4;
const MSP_BUILD_INFO: u8 = 5;
const MSP_FEATURE_CONFIG: u8 = 36;
const MSP_REBOOT: u8 = 68;
const MSP_ADVANCED_CONFIG: u8 = 90;
const MSP_STATUS: u8 = 101;
const MSP_MOTOR: u8 = 104;
const MSP_BOXIDS: u8 = 119;
const MSP_MOTOR_3D_CONFIG: u8 = 124;
const MSP_MOTOR_CONFIG: u8 = 131;
const MSP_STATUS_EX: u8 = 150;
const MSP_SET_PASSTHROUGH: u8 = 245;

// Static buffers for USB descriptors (required for passthrough mode)
static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State> = StaticCell::new();

/// USB CDC console task - handles both TX drain and RX command processing
#[embassy_executor::task]
pub async fn usb_cdc_console_task(
    driver: Driver<'static, USB_OTG_FS>,
    console: &'static ConsoleMutex,
) -> ! {
    use embassy_time::Timer;
    use embassy_futures::select::{select, Either};
    use embassy_time::Duration;

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

    // Use static buffers for USB descriptors
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 256]);
    let msos_descriptor = MSOS_DESCRIPTOR.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let state = CDC_STATE.init(State::new());

    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        msos_descriptor,
        control_buf,
    );

    // Create CDC-ACM class
    let mut class = CdcAcmClass::new(&mut builder, state, 64);

    // Build the USB device
    let mut usb = builder.build();

    // Run the USB device in the background
    let usb_fut = usb.run();

    // Create console I/O future
    let console_fut = async {
        let mut msp_parser = MspParser::new();
        let mut passthrough_mode = false;
        let mut msp_mode = false;  // Set true once MSP traffic detected
        let mut four_way = FourWayInterface::new();

        loop {
            // Wait for USB to be connected and class to be ready
            class.wait_connection().await;

            // Connection established - handle I/O
            loop {
                let mut rx_buf = [0u8; 64];

                let read_fut = class.read_packet(&mut rx_buf);
                let timeout_fut = Timer::after(Duration::from_millis(1));

                match select(read_fut, timeout_fut).await {
                    Either::First(Ok(n)) if n > 0 => {
                        if passthrough_mode {
                            // In passthrough mode - handle 4-way protocol
                            const FRAME_BUF_SIZE: usize = 280;
                            static mut FRAME_BUF: [u8; FRAME_BUF_SIZE] = [0u8; FRAME_BUF_SIZE];
                            static mut FRAME_POS: usize = 0;

                            info!("4way RX {} bytes: {:02X?}", n, &rx_buf[..n.min(16)]);

                            unsafe {
                                for &byte in &rx_buf[..n] {
                                    // Collect frame bytes
                                    if FRAME_POS == 0 && byte != 0x2F {
                                        continue;  // Wait for frame start
                                    }

                                    if FRAME_POS < FRAME_BUF_SIZE {
                                        FRAME_BUF[FRAME_POS] = byte;
                                        FRAME_POS += 1;
                                    }

                                    // Check if we have a complete frame
                                    if FRAME_POS >= 5 {
                                        let param_len = FRAME_BUF[4] as usize;
                                        let expected_len = 5 + param_len + 2;
                                        if FRAME_POS >= expected_len {
                                            // Process complete frame
                                            info!("4way frame cmd=0x{:02X} len={}", FRAME_BUF[1], FRAME_POS);
                                            let response = four_way.handle_frame(&FRAME_BUF[..FRAME_POS]);
                                            info!("4way TX: {:02X?}", response.as_slice());
                                            if !response.is_empty() {
                                                if class.write_packet(&response).await.is_err() {
                                                    info!("4way TX error");
                                                }
                                            }
                                            FRAME_POS = 0;
                                        }
                                    }
                                }
                            }
                        } else {
                            // Normal mode - handle MSP commands
                            // Debug: log received bytes
                            if n > 0 && rx_buf[0] == b'$' {
                                info!("USB RX ({} bytes): {:02X?}", n, &rx_buf[..n.min(16)]);
                            }

                            // Check for MSP commands
                            let mut msp_handled = false;
                            for &byte in &rx_buf[..n] {
                                if let Some(cmd) = msp_parser.feed(byte) {
                                    msp_handled = true;
                                    msp_mode = true;  // Disable console output once MSP detected
                                    match cmd {
                                        MSP_API_VERSION => {
                                            // MSP_PROTOCOL_VERSION=0, API_VERSION_MAJOR=1, API_VERSION_MINOR=44
                                            let response = build_msp_response(cmd, &[0, 1, 44]);
                                            info!("MSP TX cmd 1: {:02X?}", &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_FC_VARIANT => {
                                            // FLIGHT_CONTROLLER_IDENTIFIER_LENGTH = 4
                                            let response = build_msp_response(cmd, b"BTFL");
                                            info!("MSP TX cmd 2: {:02X?}", &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_FC_VERSION => {
                                            // Betaflight 4.3.2: major, minor, patch (3 bytes, no pstring)
                                            let response = build_msp_response(cmd, &[
                                                4,   // FC_VERSION_MAJOR
                                                3,   // FC_VERSION_MINOR
                                                2,   // FC_VERSION_PATCH_LEVEL
                                            ]);
                                            info!("MSP TX cmd 3: {:02X?}", &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_BOARD_INFO => {
                                            // Simplified response like ArduPilot: board ID + hw rev + type
                                            let response = build_msp_response(cmd, &[
                                                b'S', b'7', b'2', b'2',  // board identifier
                                                0, 0,                    // hardware revision
                                                0,                       // hardware type (0 = FC)
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_BUILD_INFO => {
                                            // Betaflight format: date(11) + time(8) + gitrev(7) = 26 bytes
                                            let date = b"Mar 21 2026";
                                            let time = b"12:00:00";
                                            let git_rev = b"abcdefg";
                                            let mut payload = [0u8; 26];
                                            payload[..11].copy_from_slice(date);
                                            payload[11..19].copy_from_slice(time);
                                            payload[19..26].copy_from_slice(git_rev);
                                            let response = build_msp_response(cmd, &payload);
                                            info!("MSP TX cmd 5: {:02X?}", &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_FEATURE_CONFIG => {
                                            let response = build_msp_response(cmd, &[0, 0, 0, 0]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_ADVANCED_CONFIG => {
                                            // cmd 90: gyro/motor config (10 bytes like ArduPilot)
                                            let response = build_msp_response(cmd, &[
                                                1,           // gyro_sync_denom (deprecated)
                                                4,           // pid_process_denom
                                                0,           // useContinuousUpdate
                                                6,           // motorProtocol: DSHOT300 = 6
                                                0xE0, 0x01,  // motorPwmRate = 480
                                                0xC2, 0x01,  // motorIdle = 450
                                                0,           // gyro_use_32kHz (deprecated)
                                                0,           // motorInversion
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_STATUS => {
                                            // cmd 101: status
                                            let response = build_msp_response(cmd, &[
                                                0xE8, 0x03,  // cycleTime (1000us)
                                                0, 0,        // i2cErrorCounter
                                                0x21, 0,     // sensors (ACC + GYRO)
                                                0, 0, 0, 0,  // flightModeFlags (none active)
                                                0,           // currentPidProfileIndex
                                                5, 0,        // averageSystemLoadPercent = 5
                                                0, 0,        // gyro cycle time (MSP_STATUS only)
                                                0,           // flightModeFlags extension byteCount
                                                25,          // ARMING_DISABLE_FLAGS_COUNT
                                                0, 0, 0, 0,  // armingDisableFlags = 0 (all clear, can arm)
                                                0,           // rebootRequired
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_MOTOR => {
                                            // cmd 104: 8 motor values as u16
                                            // Return 1000 (minCommand/stopped) for configured motors
                                            let response = build_msp_response(cmd, &[
                                                0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03,  // motors 1-4 = 1000
                                                0, 0, 0, 0, 0, 0, 0, 0,  // motors 5-8 = 0 (not present)
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_BOXIDS => {
                                            // cmd 119: permanent box IDs
                                            let response = build_msp_response(cmd, &[0, 27, 30, 36]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_MOTOR_3D_CONFIG => {
                                            // cmd 124: 3D config (deadband3d_low, deadband3d_high, neutral3d)
                                            let response = build_msp_response(cmd, &[
                                                0x06, 0x05,  // deadband3d_low = 1286
                                                0xFA, 0x05,  // deadband3d_high = 1530
                                                0xDC, 0x05,  // neutral3d = 1500
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_MOTOR_CONFIG => {
                                            // cmd 131: motor configuration
                                            let response = build_msp_response(cmd, &[
                                                0, 0,        // minThrottle (deprecated, was until 4.5)
                                                0xD0, 0x07,  // maxThrottle = 2000
                                                0xE8, 0x03,  // minCommand = 1000
                                                4,           // motorCount = 4
                                                14,          // motorPoleCount = 14
                                                0,           // useDshotTelemetry
                                                0,           // escSensor
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_STATUS_EX => {
                                            // cmd 150: extended status
                                            let response = build_msp_response(cmd, &[
                                                0xE8, 0x03,  // cycleTime (1000us)
                                                0, 0,        // i2cErrorCounter
                                                0x21, 0,     // sensors (ACC + GYRO)
                                                0, 0, 0, 0,  // flightModeFlags (none active)
                                                0,           // currentPidProfileIndex
                                                5, 0,        // averageSystemLoadPercent = 5
                                                3,           // PID_PROFILE_COUNT
                                                0,           // currentControlRateProfileIndex
                                                0,           // flightModeFlags extension byteCount
                                                25,          // ARMING_DISABLE_FLAGS_COUNT
                                                0, 0, 0, 0,  // armingDisableFlags = 0 (all clear)
                                                0,           // rebootRequired
                                            ]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_REBOOT => {
                                            let response = build_msp_response(cmd, &[0]);
                                            info!("MSP TX cmd {}: {:02X?}", cmd, &response);
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }
                                        }
                                        MSP_SET_PASSTHROUGH => {
                                            // cmd 245: enter passthrough mode
                                            info!("MSP cmd {} - entering passthrough", cmd);

                                            // Configure motor pins as GPIO input with pull-up
                                            // (timer outputs will be overridden by GPIO mode)
                                            init_motor_pins_gpio();

                                            let response = build_msp_response(cmd, &[4]);  // 4 ESCs
                                            if class.write_packet(&response).await.is_err() {
                                                info!("  ERROR: write failed");
                                            }

                                            passthrough_mode = true;
                                            info!("Passthrough mode active");
                                        }
                                        _ => {
                                            // Unknown command - send empty response
                                            info!("MSP cmd {} - no handler", cmd);
                                        }
                                    }
                                }
                            }

                            if passthrough_mode {
                                continue;
                            }

                            // Only process console commands if no MSP was handled
                            if !msp_handled {
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
                        }
                    }
                    Either::First(Err(EndpointError::Disabled)) => {
                        // USB disconnected
                        break;
                    }
                    _ => {
                        // Timeout - continue
                    }
                }

                // Drain output ring buffer to TX (skip in MSP/passthrough mode)
                if !passthrough_mode && !msp_mode {
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
                                break;
                            }
                            Err(_) => {}
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

/// Build MSP response frame
fn build_msp_response(cmd: u8, payload: &[u8]) -> heapless::Vec<u8, 128> {
    let mut response: heapless::Vec<u8, 128> = heapless::Vec::new();
    let _ = response.push(b'$');
    let _ = response.push(b'M');
    let _ = response.push(b'>');
    let _ = response.push(payload.len() as u8);
    let _ = response.push(cmd);

    let mut checksum = payload.len() as u8 ^ cmd;
    for &b in payload {
        let _ = response.push(b);
        checksum ^= b;
    }
    let _ = response.push(checksum);

    response
}
