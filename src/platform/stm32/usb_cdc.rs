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

// MSP command IDs
const MSP_API_VERSION: u8 = 1;
const MSP_FC_VARIANT: u8 = 2;
const MSP_FC_VERSION: u8 = 3;
const MSP_BOARD_INFO: u8 = 4;
const MSP_BUILD_INFO: u8 = 5;
const MSP_FEATURE_CONFIG: u8 = 36;
const MSP_REBOOT: u8 = 68;
const MSP_STATUS: u8 = 90;
const MSP_STATUS_EX: u8 = 101;
const MSP_MOTOR_CONFIG: u8 = 119;
const MSP_MOTOR: u8 = 124;
const MSP_BOXIDS: u8 = 104;
const MSP_ADVANCED_CONFIG: u8 = 131;
const MSP_PASSTHROUGH_ESC_4WAY: u8 = 0xFF;

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
                                            let response = four_way.handle_frame(&FRAME_BUF[..FRAME_POS]);
                                            if !response.is_empty() {
                                                let _ = class.write_packet(&response).await;
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
                                            // MSP_PROTOCOL_VERSION=0, API_VERSION_MAJOR=1, API_VERSION_MINOR=48
                                            let response = build_msp_response(cmd, &[0, 1, 48]);
                                            info!("MSP TX cmd 1");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_FC_VARIANT => {
                                            // FLIGHT_CONTROLLER_IDENTIFIER_LENGTH = 4
                                            let response = build_msp_response(cmd, b"BTFL");
                                            info!("MSP TX cmd 2");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_FC_VERSION => {
                                            // year-2000, month, patch, then pstring version
                                            let response = build_msp_response(cmd, &[
                                                24,  // FC_VERSION_YEAR - 2000
                                                5,   // FC_VERSION_MONTH
                                                0,   // FC_VERSION_PATCH_LEVEL
                                                5, b'4', b'.', b'5', b'.', b'0',  // pstring "4.5.0"
                                            ]);
                                            info!("MSP TX cmd 3");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_BOARD_INFO => {
                                            // Full BOARD_INFO for API 1.48
                                            let response = build_msp_response(cmd, &[
                                                b'S', b'7', b'2', b'2',  // boardIdentifier (4)
                                                0, 0,                    // hardwareRevision (2)
                                                0,                       // type: 0=FC (1)
                                                1,                       // targetCapabilities: VCP (1)
                                                11,                      // targetName length
                                                b'S', b'T', b'M', b'3', b'2', b'F', b'7', b'2', b'2', b'R', b'E',
                                                0,                       // boardName length (empty)
                                                0,                       // manufacturerId length (empty)
                                                // signature (32 bytes, all zeros)
                                                0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
                                                0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,
                                                4,                       // mcuTypeId (4=MCU_TYPE_F722)
                                                1,                       // configurationState (1=CONFIGURED)
                                                0xE8, 0x03,              // sampleRateHz (1000)
                                                0, 0, 0, 0,              // configurationProblems (none)
                                                0,                       // spiDeviceCount
                                                0,                       // i2cDeviceCount
                                            ]);
                                            info!("MSP TX cmd 4");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_BUILD_INFO => {
                                            // date(11) + time(8) + gitrev(7) = 26 bytes exactly
                                            let response = build_msp_response(cmd, b"Mar 21 202612:00:00abcdefg");
                                            info!("MSP TX cmd 5");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_FEATURE_CONFIG => {
                                            // Response: feature mask (4 bytes)
                                            let response = build_msp_response(cmd, &[0, 0, 0, 0]);
                                            info!("MSP TX cmd 36");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_STATUS => {
                                            // MSP_STATUS format - same as STATUS_EX but with gyro cycle time instead of profiles
                                            let response = build_msp_response(cmd, &[
                                                0xE8, 0x03,  // cycleTime (1000us)
                                                0, 0,        // i2cErrorCounter
                                                0x20, 0,     // sensors (gyro = bit 5)
                                                0, 0, 0, 0,  // flightModeFlags (disarmed)
                                                0,           // currentPidProfileIndex
                                                0, 0,        // averageSystemLoadPercent
                                                0, 0,        // gyro cycle time (MSP_STATUS uses this)
                                                0,           // flightModeFlags extension byteCount (0 extra bytes)
                                                29,          // ARMING_DISABLE_FLAGS_COUNT
                                                0, 0, 0, 0,  // armingDisableFlags = 0 (all clear)
                                                0,           // rebootRequired = false
                                                25, 0,       // coreTemperature = 25C
                                                3,           // CONTROL_RATE_PROFILE_COUNT
                                            ]);
                                            info!("MSP TX cmd 90");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_MOTOR_CONFIG => {
                                            // Betaflight MSP_MOTOR_CONFIG format
                                            let response = build_msp_response(cmd, &[
                                                0, 0,        // minThrottle = 0 (was removed after API 4.5)
                                                0xD0, 0x07,  // maxThrottle = 2000
                                                0xE8, 0x03,  // minCommand = 1000
                                                4,           // getMotorCount() = 4
                                                14,          // motorPoleCount = 14
                                                0,           // useDshotTelemetry = false
                                                0,           // escSensor feature = false
                                            ]);
                                            info!("MSP TX cmd 119");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_STATUS_EX => {
                                            // MSP_STATUS_EX - matches Betaflight format
                                            let response = build_msp_response(cmd, &[
                                                0xE8, 0x03,  // cycleTime (1000us)
                                                0, 0,        // i2cErrorCounter
                                                0x20, 0,     // sensors (gyro = bit 5)
                                                0, 0, 0, 0,  // flightModeFlags (disarmed)
                                                0,           // currentPidProfileIndex
                                                0, 0,        // averageSystemLoadPercent
                                                3,           // PID_PROFILE_COUNT
                                                0,           // currentControlRateProfileIndex
                                                0,           // flightModeFlags extension byteCount (0 extra bytes)
                                                29,          // ARMING_DISABLE_FLAGS_COUNT = 29
                                                0, 0, 0, 0,  // armingDisableFlags = 0 (all clear)
                                                0,           // rebootRequired = false
                                                25, 0,       // coreTemperature = 25C
                                                3,           // CONTROL_RATE_PROFILE_COUNT
                                            ]);
                                            info!("MSP TX cmd 101");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_MOTOR => {
                                            // 8 motors * u16 = 16 bytes, all stopped (value 0)
                                            let response = build_msp_response(cmd, &[
                                                0, 0, 0, 0, 0, 0, 0, 0,  // motors 1-4 stopped
                                                0, 0, 0, 0, 0, 0, 0, 0,  // motors 5-8 stopped
                                            ]);
                                            info!("MSP TX cmd 124");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_BOXIDS => {
                                            // Empty box IDs list
                                            let response = build_msp_response(cmd, &[]);
                                            info!("MSP TX cmd 104");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_ADVANCED_CONFIG => {
                                            // Betaflight MSP_ADVANCED_CONFIG format
                                            let response = build_msp_response(cmd, &[
                                                1,           // gyro_sync_denom (deprecated, always 1)
                                                1,           // pid_process_denom
                                                0,           // useContinuousUpdate
                                                5,           // motorProtocol (5 = DSHOT300)
                                                0xE8, 0x03,  // motorPwmRate = 1000
                                                0, 0,        // motorIdle = 0
                                                0,           // gyro_use_32kHz (deprecated)
                                                0,           // motorInversion
                                                0,           // gyro_to_use (deprecated)
                                                0,           // gyro_high_fsr
                                                48,          // gyroMovementCalibrationThreshold
                                                0x78, 0x00,  // gyroCalibrationDuration = 120
                                                0, 0,        // gyro_offset_yaw
                                                2,           // checkOverflow (GYRO_OVERFLOW_CHECK_ALL)
                                                0,           // debug_mode
                                                4,           // DEBUG_COUNT
                                            ]);
                                            info!("MSP TX cmd 131");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_REBOOT => {
                                            // Just ACK, don't actually reboot
                                            let response = build_msp_response(cmd, &[]);
                                            info!("MSP TX cmd 68");
                                            let _ = class.write_packet(&response).await;
                                        }
                                        MSP_PASSTHROUGH_ESC_4WAY => {
                                            info!("BLHeli passthrough mode entered");

                                            // Disable DShot/motor control timers
                                            unsafe {
                                                let tim3_cr1 = 0x4000_0400 as *mut u32;
                                                core::ptr::write_volatile(tim3_cr1, 0);
                                                let tim1_cr1 = 0x4001_0000 as *mut u32;
                                                core::ptr::write_volatile(tim1_cr1, 0);
                                            }

                                            // Configure motor pins as GPIO input with pull-up
                                            init_motor_pins_gpio();

                                            // Send MSP response with ESC count
                                            let response = build_msp_response(cmd, &[4]);
                                            let _ = class.write_packet(&response).await;
                                            Timer::after(Duration::from_millis(10)).await;

                                            passthrough_mode = true;
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
