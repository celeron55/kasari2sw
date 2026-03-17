#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::Timer;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};

extern crate alloc;

#[global_allocator]
static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize heap (RP2350 has 520KB SRAM)
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 256 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    info!("Kasari2sw RP2350 Pico 2 W starting...");

    let p = embassy_rp::init(Default::default());

    info!("RP2350 initialized");

    // Simple GPIO LED blink test on GP25 (near onboard LED location)
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("Starting LED blink test on GP25");

    // Blink LED
    loop {
        info!("LED on");
        led.set_high();
        Timer::after_millis(500).await;

        info!("LED off");
        led.set_low();
        Timer::after_millis(500).await;
    }
}

// TODO: CYW43 WiFi initialization will be added in Phase 6
// The Pico 2 W uses CYW43439 for WiFi and the onboard LED.
// For now, we're using a simple GPIO pin to verify basic firmware works.
