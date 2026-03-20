use core::fmt::Write;
use embassy_stm32::usart::{BufferedUartRx, BufferedUartTx};
use heapless::String;
use log::{LevelFilter, Metadata, Record};
use ringbuffer::RingBuffer;

use crate::console::{get_uart_console, get_usb_console, ConsoleMutex};

// Import traits for BufferedUart read/write
use embedded_io_async::Read as AsyncRead;
use embedded_io_async::Write as AsyncWrite;

pub struct Logger;

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let level_str = match record.level() {
                log::Level::Error => "ERROR",
                log::Level::Warn => "WARN",
                log::Level::Info => "INFO",
                log::Level::Debug => "DEBUG",
                log::Level::Trace => "TRACE",
            };

            let mut buf: String<256> = String::new();
            if writeln!(buf, "[{}] {}\r", level_str, record.args()).is_ok() {
                let data = buf.as_bytes();

                // Write to both consoles (only if in Log mode - handled by write_log)
                cortex_m::interrupt::free(|cs| {
                    get_uart_console()
                        .borrow(cs)
                        .borrow_mut()
                        .write_log(data);
                    get_usb_console()
                        .borrow(cs)
                        .borrow_mut()
                        .write_log(data);
                });
            }
        }
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

pub fn init_logger() {
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);
}

/// UART console task - handles TX drain and RX command processing
/// Uses interrupt-driven BufferedUart for reliable high-speed RX
#[embassy_executor::task]
pub async fn uart_console_task(
    mut tx: BufferedUartTx<'static>,
    mut rx: BufferedUartRx<'static>,
    console: &'static ConsoleMutex,
) -> ! {
    use embassy_futures::select::{select, Either};
    use embassy_time::{Duration, Timer};

    loop {
        // Wait for either RX data or timeout (for TX drain)
        let mut rx_buf = [0u8; 32];
        let rx_fut = AsyncRead::read(&mut rx, &mut rx_buf);
        let timeout_fut = Timer::after(Duration::from_millis(10));

        match select(rx_fut, timeout_fut).await {
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
            _ => {
                // Timeout or error - continue with TX drain
            }
        }

        // Drain output ring buffer to TX
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
            let _ = AsyncWrite::write_all(&mut tx, &pending).await;
        }
    }
}
