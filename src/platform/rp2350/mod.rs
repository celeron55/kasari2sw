#![no_std]
#![no_main]

mod pins;
mod sensors;
mod logging;
// mod motors;

use log::info;
use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{PIO0, DMA_CH0, UART1, UART0},
    pio::{self, Pio},
    uart::{BufferedUart, Config as UartConfig, BufferedInterruptHandler},
};
use embassy_time::Timer;
use static_cell::StaticCell;

extern crate alloc;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[link_section = ".modem_firmware"]
static MODEM_FIRMWARE: &[u8] = include_bytes!("../../../cyw43-firmware/43439A0.bin");

#[link_section = ".modem_firmware"]
static COUNTRY_LOCALE_MATRIX: &[u8] = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
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

    let p = embassy_rp::init(Default::default());

    // Initialize CYW43 WiFi chip (controls onboard LED)
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);

    const CLOCK_DIVIDER: u16 = 78;
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

    spawner.spawn(wifi_task(runner)).unwrap();

    control.init(COUNTRY_LOCALE_MATRIX).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    info!("CYW43 initialized");

    // Initialize UART0 for logging (115200 baud) - buffered for non-blocking
    static UART0_TX_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    static UART0_RX_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    
    let mut uart0_config = UartConfig::default();
    uart0_config.baudrate = 115200;
    let uart0 = BufferedUart::new(
        p.UART0,
        p.PIN_0,   // TX (GP0)
        p.PIN_1,   // RX (GP1)
        Irqs,
        UART0_TX_BUFFER.init([0; 256]),
        UART0_RX_BUFFER.init([0; 256]),
        uart0_config,
    );
    let (uart0_tx, _uart0_rx) = uart0.split();
    
    // Initialize logger - logs go to UART0 via BufferedUartTx
    logging::init_logger(uart0_tx);
    info!("Logger initialized - logs streaming to UART0 (GP0 TX, GP1 RX, 115200 baud, buffered)");
    info!("TCP log server not yet implemented - requires embassy-net WiFi integration");

    // Initialize buffered UART1 for LIDAR (921600 baud)
    static UART1_TX_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    static UART1_RX_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 921600;
    let uart = BufferedUart::new(
        p.UART1,
        p.PIN_4,  // TX
        p.PIN_5,  // RX
        Irqs,
        UART1_TX_BUFFER.init([0; 256]),
        UART1_RX_BUFFER.init([0; 256]),
        uart_config,
    );
    let (_tx, rx) = uart.split();

    info!("Peripherals initialized, starting LIDAR task");

    // Spawn LIDAR task
    spawner.spawn(sensors::lidar_task(rx)).unwrap();

    // Main loop - just blink LED for now
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
