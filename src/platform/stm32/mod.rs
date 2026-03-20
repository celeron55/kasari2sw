#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    usart::{Config as UartConfig, InterruptHandler as UsartInterruptHandler, Uart, RingBufferedUartRx},
    Config,
};
use static_cell::StaticCell;
use embassy_time::Timer;
use log::info;

extern crate alloc;

mod sensors;
mod logging;
mod panic_uart;
mod usb_cdc;
pub mod dshot;
pub mod dshot_dma;
// mod debug_uart;  // Commented out - not used

use kasarisw::shared::kasari::{InputEvent, MainLogic, MotorModulator};

bind_interrupts!(struct Irqs {
    UART4 => UsartInterruptHandler<peripherals::UART4>;
    USART2 => UsartInterruptHandler<peripherals::USART2>;
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    panic_uart::init();
    panic_uart::write("\r\n\r\n=== PANIC ===\r\n");

    if let Some(location) = info.location() {
        panic_uart::write("Location: ");
        panic_uart::write(location.file());
        panic_uart::write(":");
        panic_uart::write_decimal(location.line() as u32);
        panic_uart::write("\r\n");
    }

    panic_uart::write("\r\nHalting...\r\n");

    loop {
        cortex_m::asm::wfi();
    }
}

// Global allocator for STM32
#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

const HEAP_SIZE: usize = 128 * 1024; // 128KB heap
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

// ESC target speed (bidirectional) to servo signal PWM duty cycle
// This function is platform-independent
const PWM_HZ: f32 = 400.0;
fn target_speed_to_pwm_duty(speed_percent: f32, duty_range: u32) -> u32 {
    let center_pwm = 0.00149 * PWM_HZ;
    let pwm_amplitude = 0.000350 * PWM_HZ;
    let duty_percent = (center_pwm * 100.0 + pwm_amplitude * speed_percent)
        .min(100.0)
        .max(-100.0);
    (duty_range * duty_percent as u32) / 100
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize heap
    unsafe {
        ALLOCATOR.init(&mut HEAP as *const u8 as usize, HEAP_SIZE);
    }

    // Configure STM32F722 system clock
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // Mamba F722APP has 8 MHz HSE crystal
        config.rcc.hse = Some(Hse {
            freq: embassy_stm32::time::Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,      // 8 MHz / 8 = 1 MHz
            mul: PllMul::MUL432,           // 1 MHz * 432 = 432 MHz
            divp: Some(PllPDiv::DIV2),     // 432 MHz / 2 = 216 MHz (SYSCLK)
            divq: Some(PllQDiv::DIV9),     // 432 MHz / 9 = 48 MHz (USB, SDIO, RNG)
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;  // 216 MHz AHB
        config.rcc.apb1_pre = APBPrescaler::DIV4; // 54 MHz APB1
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 108 MHz APB2
        config.rcc.sys = Sysclk::PLL1_P;          // Use PLL as system clock
    }

    let p = embassy_stm32::init(config);

    // Removed debug pin test loop - it was blocking DShot initialization

    // Enable overdrive mode AFTER init for 216 MHz (>180 MHz requires this)
    // Embassy might have configured clocks but not enabled overdrive
    unsafe {
        use embassy_stm32::pac;
        use embassy_stm32::pac::pwr::vals::Vos;
        // Set voltage scale 1 for high performance (required for 216 MHz)
        pac::PWR.cr1().modify(|w| w.set_vos(Vos::SCALE1));
        // Enable overdrive mode (required for >180 MHz)
        pac::PWR.cr1().modify(|w| w.set_oden(true));
        while !pac::PWR.csr1().read().odrdy() {}
        pac::PWR.cr1().modify(|w| w.set_odswen(true));
        while !pac::PWR.csr1().read().odswrdy() {}
    }

    // ============================================================================
    // UART4 LOGGING - WiFi adapter port (PA0=TX, PA1=RX, 115200 baud)
    // ============================================================================

    let mut uart4_config = UartConfig::default();
    uart4_config.baudrate = 115200;
    // Create UART4 in blocking mode (no DMA) to free DMA1 Stream2 for TIM3_CH4
    // UART4 RX can only use DMA1 Stream2, which conflicts with TIM3_CH4
    // Since we need both DShot channels, UART4 must use blocking/interrupt mode
    let uart4 = Uart::new_blocking(
        p.UART4,
        p.PA1,  // RX (interrupt mode)
        p.PA0,  // TX (interrupt mode)
        uart4_config,
    ).unwrap();

    // Initialize logger (buffered to ring buffer)
    logging::init_logger();
    info!("Kasari2sw STM32F722 starting...");
    info!("STM32F722 initialized at 216 MHz (8 MHz HSE, overdrive enabled)");

    // Spawn log drain task to send buffered logs to UART4
    spawner.spawn(logging::log_drain_task(uart4)).unwrap();
    info!("Logger initialized - logs streaming to UART4 (PA0 TX, PA1 RX, 115200 baud, blocking mode)");

    // ============================================================================
    // USB CDC LOGGING - Direct USB logging
    // ============================================================================

    use embassy_stm32::usb::Driver;
    use static_cell::StaticCell;

    static USB_EP_OUT_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();

    let mut usb_config = embassy_stm32::usb::Config::default();
    usb_config.vbus_detection = false; // No VBUS detection (PA9 used for motor)

    let usb_driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12, // D+
        p.PA11, // D-
        USB_EP_OUT_BUFFER.init([0u8; 1024]),
        usb_config,
    );

    // Spawn USB CDC log drain task
    spawner.spawn(usb_cdc::usb_cdc_log_drain_task(usb_driver)).unwrap();
    info!("USB CDC initialized - logs also streaming to USB (PA12 D+, PA11 D-)");

    // ============================================================================
    // EVENT CHANNEL - Pub/Sub for sensor events
    // ============================================================================

    use kasarisw::shared::{EventChannel, EVENT_CHANNEL};
    let event_channel = EVENT_CHANNEL.init(EventChannel::new());
    info!("Event channel initialized");

    // ============================================================================
    // LIDAR UART2 - TFA300 at 921600 baud (PA3 RX only)
    // ============================================================================

    // DMA ring buffer for high-speed UART2 reception
    static LIDAR_DMA_BUF: StaticCell<[u8; 256]> = StaticCell::new();

    let mut uart2_config = UartConfig::default();
    uart2_config.baudrate = 921600;

    // Create UART2 with DMA for efficient high-speed reception
    // USART2_RX uses DMA1_Stream5 (no conflict with DShot on Stream2/7)
    let uart2 = Uart::new(
        p.USART2,
        p.PA3,  // RX (LIDAR data in)
        p.PA2,  // TX (unused but required by HAL)
        Irqs,
        p.DMA1_CH6,  // TX DMA (unused)
        p.DMA1_CH5,  // RX DMA
        uart2_config,
    ).unwrap();

    // Split and use only RX with ring buffer
    let (_uart2_tx, uart2_rx) = uart2.split();
    let lidar_rx = uart2_rx.into_ring_buffered(LIDAR_DMA_BUF.init([0u8; 256]));

    // Get publisher for LIDAR task
    let lidar_publisher = event_channel.publisher().unwrap();
    spawner.spawn(sensors::lidar_publisher(lidar_rx, lidar_publisher)).unwrap();
    info!("LIDAR initialized - UART2 at 921600 baud (PA3 RX, DMA1_CH5)");

    // ============================================================================
    // ADXL373 ACCELEROMETER - Bit-bang SPI (100 Hz sampling)
    // ============================================================================
    // Pins: CS=PC12 (TX5), SCK=PB3 (LED), MISO=PC2 (RSSI), MOSI=PD2 (RX5)

    use embassy_stm32::gpio::{Input, Pull};

    let accel_cs = Output::new(p.PC12, Level::High, Speed::VeryHigh);
    let accel_sck = Output::new(p.PB3, Level::Low, Speed::VeryHigh);
    let accel_mosi = Output::new(p.PD2, Level::Low, Speed::VeryHigh);
    let accel_miso = Input::new(p.PC2, Pull::None);

    let accel_publisher = event_channel.publisher().unwrap();
    spawner.spawn(sensors::accel_publisher(accel_cs, accel_sck, accel_mosi, accel_miso, accel_publisher)).unwrap();
    info!("ADXL373 initialized - bit-bang SPI (CS=PC12, SCK=PB3, MISO=PC2, MOSI=PD2)");

    // ============================================================================
    // MAIN LOGIC TASK - Event processing and motor control planning
    // ============================================================================

    // Create shared state for MainLogic and MotorModulator
    use core::cell::RefCell;
    use critical_section::Mutex;

    static MAIN_LOGIC: StaticCell<Mutex<RefCell<MainLogic>>> = StaticCell::new();
    let main_logic = MAIN_LOGIC.init(Mutex::new(RefCell::new(MainLogic::new(false))));

    static MOTOR_MODULATOR: StaticCell<Mutex<RefCell<MotorModulator>>> = StaticCell::new();
    let motor_modulator = MOTOR_MODULATOR.init(Mutex::new(RefCell::new(MotorModulator::new())));

    // Get subscriber and publisher for main logic task
    let main_logic_subscriber = event_channel.subscriber().unwrap();
    let main_logic_publisher = event_channel.publisher().unwrap();
    spawner.spawn(main_logic_task(main_logic, motor_modulator, main_logic_subscriber, main_logic_publisher)).unwrap();
    info!("MainLogic task started");

    // ============================================================================
    // LED TEST - Mamba F722APP Status LEDs
    // ============================================================================

    // PC15 = GYRO (blue), PC14 = MCU (orange) - ACTIVE LOW!
    let _led_gyro = Output::new(p.PC15, Level::High, Speed::Low);  // Start OFF (HIGH = OFF)
    let mut led_mcu = Output::new(p.PC14, Level::High, Speed::Low);   // Start OFF

    info!("LED test: GYRO (blue/PC15), MCU (orange/PC14) - active LOW");

    // ============================================================================
    // TODO: PERIPHERAL INITIALIZATION (awaiting Mamba F722APP pinout)
    // ============================================================================

    // TODO: Event channel for pub/sub between sensor tasks and main logic
    // let event_channel = mk_static!(EventChannel, EventChannel::new());

    // TODO: Motor PWM (400 Hz, likely TIM3 or TIM4, motor outputs 1-2)
    // use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, Channel};
    // let motor_pwm_freq = embassy_stm32::time::Hertz(400);
    // let mut motor_pwm = SimplePwm::new(
    //     p.TIM3,
    //     Some(PwmPin::new_ch1(p.PB0, OutputType::PushPull)),  // Right motor (pin TBD)
    //     Some(PwmPin::new_ch2(p.PB1, OutputType::PushPull)),  // Left motor (pin TBD)
    //     None, None,
    //     motor_pwm_freq,
    //     Default::default(),
    // );
    // motor_pwm.enable(Channel::Ch1);
    // motor_pwm.enable(Channel::Ch2);
    // let max_duty = motor_pwm.get_max_duty();
    // let neutral_duty = target_speed_to_pwm_duty(0.0, max_duty);
    // motor_pwm.set_duty(Channel::Ch1, neutral_duty);
    // motor_pwm.set_duty(Channel::Ch2, neutral_duty);

    // TODO: LIDAR UART (TFA300 @ 115200 baud, RX only, likely USART1 or USART6)
    // bind_interrupts!(struct LidarIrqs {
    //     USART1 => embassy_stm32::usart::InterruptHandler<peripherals::USART1>;
    // });
    // let mut lidar_config = embassy_stm32::usart::Config::default();
    // lidar_config.baudrate = 115200;
    // let lidar_uart = embassy_stm32::usart::Uart::new(
    //     p.USART1,
    //     p.PA10,  // RX pin (TBD)
    //     p.PA9,   // TX pin (unused but required)
    //     LidarIrqs,
    //     p.DMA2_CH2,  // RX DMA
    //     p.DMA2_CH7,  // TX DMA (unused)
    //     lidar_config,
    // ).unwrap();
    // let (_lidar_tx, lidar_rx) = lidar_uart.split();

    // TODO: WiFi Adapter UART (likely USART2, ESP8266/ESP32 AT firmware or transparent bridge)
    // bind_interrupts!(struct WifiIrqs {
    //     USART2 => embassy_stm32::usart::InterruptHandler<peripherals::USART2>;
    // });
    // let mut wifi_config = embassy_stm32::usart::Config::default();
    // wifi_config.baudrate = 115200;
    // let wifi_uart = embassy_stm32::usart::Uart::new(
    //     p.USART2,
    //     p.PA3,  // RX pin (TBD)
    //     p.PA2,  // TX pin (TBD)
    //     WifiIrqs,
    //     p.DMA1_CH6,  // RX DMA
    //     p.DMA1_CH7,  // TX DMA
    //     wifi_config,
    // ).unwrap();

    // TODO: Accelerometer SPI (ADXL373, 100 kHz, likely SPI1)
    // let mut spi_config = embassy_stm32::spi::Config::default();
    // spi_config.frequency = embassy_stm32::time::Hertz(100_000);
    // let accel_spi = embassy_stm32::spi::Spi::new_blocking(
    //     p.SPI1,
    //     p.PA5,  // SCK (TBD)
    //     p.PA7,  // MOSI (TBD)
    //     p.PA6,  // MISO (TBD)
    //     spi_config,
    // );
    // let accel_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);  // CS (TBD)

    // TODO: RC Receiver Input Capture (PWM measurement, likely TIM1 or TIM2)
    // use embassy_stm32::timer::input_capture::{InputCapture, CapturePin};
    // let rc_capture = InputCapture::new(
    //     p.TIM1,
    //     Some(CapturePin::new_ch1(p.PA8)),  // RC input pin (TBD)
    //     None, None, None,
    //     embassy_stm32::time::Hertz(1_000_000),  // 1 MHz for microsecond resolution
    // );

    // TODO: Battery Voltage ADC (likely ADC1)
    // let mut vbat_adc = embassy_stm32::adc::Adc::new(p.ADC1);
    // let vbat_pin = p.PC0;  // ADC input (TBD)

    // TODO: Status LED
    // let mut status_led = Output::new(p.PB2, Level::Low, Speed::Low);  // LED pin (TBD)

    // TODO: Flash storage for logging (internal flash or external SPI flash)
    // Option A: Internal flash (partition ~256KB)
    // use embassy_stm32::flash::{Flash, Blocking};
    // let flash = Flash::new_blocking(p.FLASH);
    // const LOG_FLASH_START: u32 = 0x0804_0000;  // Sector 6 onwards
    // const LOG_FLASH_SIZE: u32 = 128 * 1024;
    //
    // Option B: External SPI flash (if available on FC)
    // let flash_spi = embassy_stm32::spi::Spi::new(...);

    // ============================================================================
    // TODO: SPAWN SENSOR TASKS
    // ============================================================================

    // TODO: Spawn LIDAR publisher task
    // spawner.spawn(sensors::lidar_publisher(lidar_rx, event_channel)).unwrap();

    // TODO: Spawn accelerometer task
    // spawner.spawn(sensors::accelerometer_task(accel_spi, accel_cs, event_channel)).unwrap();

    // TODO: Spawn RC receiver task
    // spawner.spawn(sensors::rc_receiver_task(rc_capture, event_channel)).unwrap();

    // TODO: Spawn battery voltage monitoring task
    // spawner.spawn(battery_monitor_task(vbat_adc, vbat_pin, event_channel)).unwrap();

    // TODO: Spawn WiFi adapter initialization task
    // spawner.spawn(wifi_init_task(wifi_uart)).unwrap();

    // TODO: Spawn TCP listener task (event streaming over WiFi)
    // spawner.spawn(tcp_listener_task(event_channel)).unwrap();

    // TODO: Spawn main logic task (MainLogic event processing)
    // let main_logic = mk_static!(MainLogic, MainLogic::new(...));
    // spawner.spawn(main_logic_task(main_logic, event_channel, motor_pwm)).unwrap();

    // ============================================================================
    // MOTOR UPDATE TASK - DShot output with MotorModulator at 500 Hz
    // ============================================================================

    spawner.spawn(motor_update_task(motor_modulator)).unwrap();
    info!("Motor update task started");

    // ============================================================================
    // MAIN LOOP - Heartbeat and status monitoring
    // ============================================================================

    loop {
        // Toggle MCU LED as heartbeat
        led_mcu.toggle();
        Timer::after_millis(500).await;
    }
}

/// Convert RPM to bidirectional DShot throttle value
/// Input: RPM (-2000 to +2000 typical range)
/// Output: DShot throttle (0=off, 48-1047=reverse, 1048=center, 1049-2047=forward)
fn rpm_to_dshot_throttle(rpm: f32) -> u16 {
    const MAX_RPM: f32 = 2000.0;
    const DSHOT_CENTER: u16 = 1048;
    const DSHOT_RANGE: u16 = 999; // 1047 - 48 or 2047 - 1049

    if rpm.abs() < 10.0 {
        // Below threshold, send zero (motors off)
        return 0;
    }

    let normalized = (rpm / MAX_RPM).clamp(-1.0, 1.0);
    let offset = (normalized * DSHOT_RANGE as f32) as i16;

    // Bidirectional DShot: 1048 is center (stopped)
    // > 1048 = forward, < 1048 = reverse
    let throttle = (DSHOT_CENTER as i16 + offset) as u16;
    throttle.clamp(48, 2047)
}

// ============================================================================
// MOTOR UPDATE TASK - Runs at 500 Hz, reads MotorModulator and sends DShot
// ============================================================================

#[embassy_executor::task]
async fn motor_update_task(
    motor_modulator: &'static critical_section::Mutex<core::cell::RefCell<MotorModulator>>,
) {
    info!("Motor update task running");

    // Initialize DShot DMA driver
    let mut dshot_dma = dshot_dma::DShotDma::new();
    dshot_dma.init();

    // ESC arming sequence (throttle=0 for 300ms)
    info!("Sending ESC arming sequence (throttle=0 for 300ms)...");
    let arm_start = embassy_time::Instant::now();
    while arm_start.elapsed().as_millis() < 300 {
        let frame = dshot::throttle_to_dshot_frame(0, false);
        dshot_dma.send_frame(frame, frame);
    }
    info!("ESC armed");

    // Motor update loop at 500 Hz (2ms interval)
    let mut heartbeat_counter = 0u32;

    loop {
        let ts = embassy_time::Instant::now().as_micros() as u64;

        // Get motor speeds from MotorModulator
        let (left_rpm, right_rpm) = critical_section::with(|cs| {
            let mut modulator = motor_modulator.borrow(cs).borrow_mut();
            modulator.step(ts)
        });

        // Convert RPM to DShot throttle values
        let left_throttle = rpm_to_dshot_throttle(left_rpm);
        let right_throttle = rpm_to_dshot_throttle(right_rpm);

        let left_frame = dshot::throttle_to_dshot_frame(left_throttle, false);
        let right_frame = dshot::throttle_to_dshot_frame(right_throttle, false);
        dshot_dma.send_frame(left_frame, right_frame);

        // Heartbeat log every ~2 seconds (1000 iterations at 500Hz)
        heartbeat_counter += 1;
        if heartbeat_counter >= 1000 {
            heartbeat_counter = 0;
            info!("Motor: L={:.0} R={:.0} RPM, thr={}/{}", left_rpm, right_rpm, left_throttle, right_throttle);
        }

        Timer::after_micros(2000).await;
    }
}

// ============================================================================
// MAIN LOGIC TASK - Processes sensor events and updates motor control plan
// ============================================================================

#[embassy_executor::task]
async fn main_logic_task(
    main_logic: &'static critical_section::Mutex<core::cell::RefCell<MainLogic>>,
    motor_modulator: &'static critical_section::Mutex<core::cell::RefCell<MotorModulator>>,
    mut subscriber: embassy_sync::pubsub::Subscriber<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, InputEvent, 64, 3, 6>,
    mut publisher: embassy_sync::pubsub::Publisher<'static, embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, InputEvent, 64, 3, 6>,
) {
    use embassy_time::{Duration, Instant};

    info!("MainLogic task running at ~50 Hz");

    const LOOP_INTERVAL_MS: u64 = 20; // Target 50 Hz
    const MIN_SLEEP_MS: u64 = 5;      // Minimum yield time for other tasks

    loop {
        let loop_start = Instant::now();
        let ts = loop_start.as_micros() as u64;

        critical_section::with(|cs| {
            let mut logic = main_logic.borrow(cs).borrow_mut();

            // Drain all pending events (non-blocking)
            while let Some(event) = subscriber.try_next_message_pure() {
                logic.feed_event(event);
            }

            // Run planning step every iteration (~50 Hz)
            logic.step(ts, Some(&mut publisher), false);

            // Sync MotorModulator with latest plan
            if let Some(plan) = logic.motor_control_plan.clone() {
                let mut modulator = motor_modulator.borrow(cs).borrow_mut();
                modulator.sync(ts, logic.detector.theta, plan, logic.angular_correction_total);
            } else {
                // No plan - clear modulator
                let mut modulator = motor_modulator.borrow(cs).borrow_mut();
                modulator.mcp = None;
            }
        });

        // Always yield at least 5ms to other tasks
        Timer::after_millis(MIN_SLEEP_MS).await;

        // Sleep remaining time to hit target interval (no catch-up if behind)
        let elapsed_ms = loop_start.elapsed().as_millis();
        if elapsed_ms < LOOP_INTERVAL_MS {
            Timer::after(Duration::from_millis(LOOP_INTERVAL_MS - elapsed_ms)).await;
        }
    }
}

// Note: alloc_error_handler is unstable. panic-probe will catch allocation failures.

// ============================================================================
// TODO: WIFI INITIALIZATION & TCP LISTENER
// ============================================================================
// Initializes WiFi adapter and handles TCP connections for event streaming
//
// #[embassy_executor::task]
// async fn wifi_init_task(mut wifi_uart: embassy_stm32::usart::Uart<'static>) {
//     // If ESP8266/ESP32 with AT firmware:
//     // 1. Send AT commands to configure:
//     //    - AT+CWMODE=3  (AP+STA mode)
//     //    - AT+CWSAP="kasarisw","",1,0  (open AP)
//     //    - AT+CWJAP="SSID","PASSWORD"  (join STA network)
//     //    - AT+CIPMUX=1  (enable multiple connections)
//     //    - AT+CIPSERVER=1,8080  (start TCP server)
//     //
//     // If transparent bridge:
//     // - Pre-configured, just start using UART for TCP data
//     //
//     // Implementation depends on specific WiFi adapter model on Mamba F722APP
// }

// #[embassy_executor::task]
// async fn tcp_listener_task(
//     event_channel: &'static EventChannel,
//     // wifi_uart: ...,  // For sending/receiving TCP data
// ) {
//     // Subscribe to events
//     let subscriber = event_channel.subscriber().unwrap();
//
//     // Accept TCP connections (via WiFi adapter)
//     loop {
//         // Wait for connection (adapter-specific)
//         // ...
//
//         // Stream events to client
//         loop {
//             let event = subscriber.next_message_pure().await;
//
//             // Serialize event to binary format (tag XOR 0x5555 + payload)
//             // Send over WiFi UART
//             // ...
//
//             // Also handle incoming commands:
//             // - WifiControl events (mode switching, manual control)
//         }
//     }
// }

// ============================================================================
// TODO: FLASH LOGGER TASK
// ============================================================================
// Buffers events and writes to flash storage periodically
//
// #[embassy_executor::task]
// async fn flash_logger_task(
//     mut flash: embassy_stm32::flash::Flash<'static>,
//     event_channel: &'static EventChannel,
// ) {
//     use embassy_time::{Duration, Timer};
//     use embedded_storage::nor_flash::NorFlash;
//
//     let subscriber = event_channel.subscriber().unwrap();
//     let mut buffer = [0u8; 4096];  // 4KB buffer
//     let mut pos = 0;
//
//     loop {
//         // Collect events until buffer is full or timeout
//         match embassy_time::with_timeout(Duration::from_secs(1), subscriber.next_message_pure()).await {
//             Ok(event) => {
//                 // Serialize event to buffer
//                 // ...
//                 pos += serialized_size;
//
//                 // If buffer full, flush to flash
//                 if pos >= buffer.len() - 128 {
//                     // flash.write(LOG_FLASH_START + offset, &buffer[..pos]).unwrap();
//                     pos = 0;
//                 }
//             }
//             Err(_) => {
//                 // Timeout, flush partial buffer
//                 if pos > 0 {
//                     // flash.write(LOG_FLASH_START + offset, &buffer[..pos]).unwrap();
//                     pos = 0;
//                 }
//             }
//         }
//     }
// }
