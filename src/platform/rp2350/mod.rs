#![no_std]
#![no_main]

use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{PIO0, DMA_CH0},
    pio::{self, Pio},
};
use embassy_time::Timer;
use static_cell::StaticCell;
use defmt::*;
use {defmt_rtt as _, panic_probe as _};

extern crate alloc;

#[link_section = ".modem_firmware"]
static MODEM_FIRMWARE: &[u8] = include_bytes!("../../../cyw43-firmware/43439A0.bin");

#[link_section = ".modem_firmware"]
static COUNTRY_LOCALE_MATRIX: &[u8] = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[global_allocator]
static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize heap (RP2350 has 520KB SRAM)
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 256 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    info!("Kasari2sw RP2350 Pico 2 W starting...");

    let p = embassy_rp::init(Default::default());

    // Initialize CYW43 WiFi chip (controls onboard LED)
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);

    const CLOCK_DIVIDER: u16 = 78; // ~1.6MHz SPI clock for CYW43
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        CLOCK_DIVIDER.into(),
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, MODEM_FIRMWARE).await;

    // Spawn WiFi task (required for CYW43 to function)
    spawner.spawn(wifi_task(runner)).unwrap();

    // Initialize CYW43
    control.init(COUNTRY_LOCALE_MATRIX).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    info!("CYW43 initialized, LED ready");

    // Blink onboard LED (GPIO 0 on CYW43)
    loop {
        info!("LED on");
        control.gpio_set(0, true).await;
        Timer::after_millis(500).await;

        info!("LED off");
        control.gpio_set(0, false).await;
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}
