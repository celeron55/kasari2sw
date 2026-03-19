#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    usart::{Config as UartConfig, InterruptHandler as UsartInterruptHandler, Uart},
    Config,
};
use embassy_time::Timer;
use log::info;

extern crate alloc;

mod sensors;
mod logging;
mod panic_uart;

bind_interrupts!(struct Irqs {
    UART4 => UsartInterruptHandler<peripherals::UART4>;
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
    let uart4 = Uart::new(
        p.UART4,
        p.PA1,  // RX
        p.PA0,  // TX
        Irqs,
        p.DMA1_CH4,  // TX DMA
        p.DMA1_CH2,  // RX DMA
        uart4_config,
    ).unwrap();

    // Initialize logger (buffered to ring buffer)
    logging::init_logger();
    info!("Kasari2sw STM32F722 starting...");
    info!("STM32F722 initialized at 216 MHz (8 MHz HSE, overdrive enabled)");

    // Spawn log drain task to send buffered logs to UART4
    spawner.spawn(logging::log_drain_task(uart4)).unwrap();
    info!("Logger initialized - logs streaming to UART4 (PA0 TX, PA1 RX, 115200 baud, DMA)");

    // ============================================================================
    // LED TEST - Mamba F722APP Status LEDs
    // ============================================================================

    // PC15 = GYRO (blue), PC14 = MCU (orange) - ACTIVE LOW!
    let mut led_gyro = Output::new(p.PC15, Level::High, Speed::Low);  // Start OFF (HIGH = OFF)
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

    // TODO: Spawn motor update task (200 Hz, reads MotorModulator and updates PWM)
    // spawner.spawn(motor_update_task(motor_pwm, main_logic)).unwrap();

    // TODO: Spawn flash logger task
    // spawner.spawn(flash_logger_task(flash, event_channel)).unwrap();

    // ============================================================================
    // CURRENT STATUS: LED blink test
    // ============================================================================

    info!("LED test: GYRO (blue) off, MCU (orange) blinking");

    // GYRO (blue) off, MCU (orange) toggling at 1 Hz
    // Active-low: HIGH = OFF, LOW = ON
    led_gyro.set_high();  // OFF

    let mut counter = 0u32;
    loop {
        led_mcu.toggle();
        info!("Heartbeat {} - LED toggled", counter);
        counter = counter.wrapping_add(1);
        Timer::after_millis(500).await;
    }
}

// Note: alloc_error_handler is unstable. panic-probe will catch allocation failures.

// ============================================================================
// TODO: MAIN LOGIC TASK
// ============================================================================
// Processes events from sensor tasks and updates robot state
//
// #[embassy_executor::task]
// async fn main_logic_task(
//     main_logic: &'static Mutex<RefCell<kasari::MainLogic>>,
//     event_channel: &'static EventChannel,
//     // motor_pwm: ...,  // Reference to motor PWM for direct updates
// ) {
//     let subscriber = event_channel.subscriber().unwrap();
//
//     loop {
//         // Wait for next event
//         let event = subscriber.next_message_pure().await;
//
//         // Process event through MainLogic
//         critical_section::with(|cs| {
//             let mut logic = main_logic.borrow(cs).borrow_mut();
//             logic.handle_input_event(event);
//
//             // MainLogic updates internal state:
//             // - ObjectDetector binning
//             // - DetectionResult (walls, objects, open spaces)
//             // - MotorModulator planning (rotation_speed, movement vector)
//             // - Mode switching (manual/autonomous)
//         });
//     }
// }

// ============================================================================
// TODO: MOTOR UPDATE TASK
// ============================================================================
// Runs at 200 Hz, reads MotorModulator state and updates PWM duty cycles
//
// #[embassy_executor::task]
// async fn motor_update_task(
//     mut motor_pwm: SimplePwm<'static>,
//     main_logic: &'static Mutex<RefCell<kasari::MainLogic>>,
// ) {
//     use embassy_time::{Duration, Ticker};
//     use embassy_stm32::timer::simple_pwm::Channel;
//
//     const MOTOR_UPDATE_HZ: u64 = 200;
//     let mut ticker = Ticker::every(Duration::from_hz(MOTOR_UPDATE_HZ));
//
//     let max_duty = motor_pwm.get_max_duty();
//
//     loop {
//         ticker.next().await;
//
//         // Read motor state from MainLogic
//         let (right_speed, left_speed) = critical_section::with(|cs| {
//             let logic = main_logic.borrow(cs).borrow();
//
//             // Get current theta and modulator state
//             let theta = logic.current_theta();
//             let modulator = logic.motor_modulator();
//
//             // Calculate differential motor speeds
//             // modulator.calculate(theta) returns (rotation_speed, amplitude, phase)
//             let (rotation_speed, amplitude, phase) = modulator.calculate(theta);
//
//             // Differential modulation formula:
//             // right_motor = base_rpm + amplitude * cos(theta - phase)
//             // left_motor = base_rpm - amplitude * cos(theta - phase)
//             let cos_val = libm::cosf(theta - phase);
//             let right_speed = rotation_speed + amplitude * cos_val;
//             let left_speed = rotation_speed - amplitude * cos_val;
//
//             (right_speed, left_speed)
//         });
//
//         // Convert speeds to PWM duty cycles
//         let right_duty = target_speed_to_pwm_duty(right_speed, max_duty);
//         let left_duty = target_speed_to_pwm_duty(left_speed, max_duty);
//
//         // Update PWM
//         motor_pwm.set_duty(Channel::Ch1, right_duty);
//         motor_pwm.set_duty(Channel::Ch2, left_duty);
//     }
// }

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
