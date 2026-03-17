// TODO: Use BufferedUartTx instead of blocking UART for non-blocking logging
// Current implementation uses blocking_write which blocks the interrupt context
// See: https://docs.rs/embassy-rp/0.9.0/embassy_rp/uart/index.html
use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use embassy_rp::uart::BufferedUartTx;
use heapless::String;
use log::{LevelFilter, Metadata, Record};

static LOG_UART: Mutex<RefCell<Option<BufferedUartTx>>> = Mutex::new(RefCell::new(None));

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
                cortex_m::interrupt::free(|cs| {
                    if let Ok(mut uart_ref) = LOG_UART.borrow(cs).try_borrow_mut() {
                        if let Some(ref mut u) = *uart_ref {
                            let data = buf.as_bytes();
                            let mut written = 0;
                            while written < data.len() {
                                match u.blocking_write(&data[written..]) {
                                    Ok(n) => written += n,
                                    Err(_) => break,
                                }
                            }
                        }
                    }
                });
            }
        }
    }

    fn flush(&self) {}
}

static LOGGER: Logger = Logger;

pub fn init_logger(uart: BufferedUartTx) {
    cortex_m::interrupt::free(|cs| {
        LOG_UART.borrow(cs).replace(Some(uart));
    });
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);
}
