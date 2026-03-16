#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    Config,
};
use embassy_time::Timer;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};

extern crate alloc;
use core::alloc::Layout;

mod sensors;
mod shared;

use shared::kasari;

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
async fn main(_spawner: Spawner) {
    // Initialize heap
    unsafe {
        ALLOCATOR.init(&mut HEAP as *const u8 as usize, HEAP_SIZE);
    }

    info!("Kasari2sw STM32F722 starting...");

    // Configure STM32F722 system clock
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: embassy_stm32::time::Hertz(25_000_000), // 25 MHz HSE on Mamba F722APP
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,     // 25 MHz / 25 = 1 MHz
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

    info!("STM32F722 initialized at 216 MHz");

    // TODO: Pin mapping needs to be determined from Mamba F722APP schematic
    // For now, just create a simple LED blink on a common pin
    // Most STM32 boards have LED on PC13, but Mamba might differ

    info!("Minimal STM32 firmware running - waiting for pinout info");
    info!("Next steps:");
    info!("1. Get Mamba F722APP pinout/schematic");
    info!("2. Identify: Motor PWM pins, LIDAR UART, WiFi UART, SPI, ADC, LED");
    info!("3. Implement peripheral initialization");
    info!("4. Port sensor tasks from ESP32 to STM32");

    // Simple loop to keep the firmware alive
    loop {
        Timer::after_millis(1000).await;
        info!("Heartbeat: {}", embassy_time::Instant::now().as_millis());
    }
}

// Note: alloc_error_handler is unstable. panic-probe will catch allocation failures.
