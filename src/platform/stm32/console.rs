//! Console abstraction for UART4 and USB CDC with output mode switching.
//!
//! Supports three output modes:
//! - Log: System log messages (via log crate)
//! - Event: Human-readable event format (JSON arrays)
//! - BinEvent: Binary event format for compact transmission

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use kasarisw::shared::kasari::InputEvent;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use static_cell::StaticCell;

pub const CONSOLE_RING_SIZE: usize = 4096;
pub const FLOW_CONTROL_THRESHOLD: usize = 3072; // 75% full - drop events above this

/// Output mode for the console
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OutputMode {
    /// System log messages only
    Log,
    /// Human-readable event format (JSON arrays)
    Event,
    /// Binary event format (compact)
    BinEvent,
}

impl Default for OutputMode {
    fn default() -> Self {
        OutputMode::Log
    }
}

/// ESC control mode for testing
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EscMode {
    /// Normal operation - use MotorModulator
    Normal,
    /// Manual override with raw throttle values (left, right)
    Override(u16, u16),
    /// Stop sending DShot frames entirely
    Off,
}

impl Default for EscMode {
    fn default() -> Self {
        EscMode::Normal
    }
}

/// Console state containing output buffer, mode, and command accumulator
pub struct ConsoleState {
    pub output_ring: ConstGenericRingBuffer<u8, CONSOLE_RING_SIZE>,
    pub mode: OutputMode,
    pub cmd_buffer: heapless::Vec<u8, 64>,
}

impl ConsoleState {
    pub fn new() -> Self {
        Self {
            output_ring: ConstGenericRingBuffer::new(),
            mode: OutputMode::Log,
            cmd_buffer: heapless::Vec::new(),
        }
    }

    /// Returns current buffer fill level as a fraction (0.0 - 1.0)
    pub fn fill_level(&self) -> f32 {
        self.output_ring.len() as f32 / CONSOLE_RING_SIZE as f32
    }

    /// Returns true if buffer has room for events (below flow control threshold)
    pub fn can_accept_events(&self) -> bool {
        self.output_ring.len() < FLOW_CONTROL_THRESHOLD
    }

    /// Write bytes to output ring buffer
    pub fn write_bytes(&mut self, data: &[u8]) {
        self.output_ring.extend(data.iter().copied());
    }

    /// Write a log message (only in Log mode)
    pub fn write_log(&mut self, msg: &[u8]) {
        if self.mode == OutputMode::Log {
            self.write_bytes(msg);
        }
    }

    /// Write an event (only in Event/BinEvent modes, with flow control)
    pub fn write_event(&mut self, event: &InputEvent) {
        if !self.can_accept_events() {
            return; // Drop event - buffer too full
        }

        match self.mode {
            OutputMode::Log => {} // No events in log mode
            OutputMode::Event => {
                let mut buf = heapless::Vec::<u8, 256>::new();
                format_event_human(event, &mut buf);
                self.write_bytes(&buf);
                self.write_bytes(b"\r\n");
            }
            OutputMode::BinEvent => {
                let mut buf = heapless::Vec::<u8, 128>::new();
                serialize_event(event, &mut buf);
                self.write_bytes(&buf);
            }
        }
    }

    /// Process a received byte, returning a command if complete
    pub fn process_rx_byte(&mut self, byte: u8) -> Option<heapless::String<64>> {
        if byte == b'\n' || byte == b'\r' {
            if self.cmd_buffer.is_empty() {
                return None;
            }
            // Convert to string and clear buffer
            let cmd = core::str::from_utf8(&self.cmd_buffer)
                .ok()
                .and_then(|s| heapless::String::try_from(s).ok());
            self.cmd_buffer.clear();
            cmd
        } else if self.cmd_buffer.len() < self.cmd_buffer.capacity() {
            let _ = self.cmd_buffer.push(byte);
            None
        } else {
            None // Buffer full, ignore
        }
    }

    /// Execute a command, returning response string
    pub fn execute_command(&mut self, cmd: &str) -> heapless::String<128> {
        let mut response: heapless::String<128> = heapless::String::new();

        // Split command into parts (simple whitespace split)
        let cmd = cmd.trim();
        let mut parts = cmd.split_whitespace();
        let command = parts.next();
        let arg = parts.next();

        match command {
            Some("out") => match arg {
                Some("log") => {
                    self.mode = OutputMode::Log;
                    let _ = response.push_str("OK: mode=log\r\n");
                }
                Some("event") => {
                    self.mode = OutputMode::Event;
                    let _ = response.push_str("OK: mode=event\r\n");
                }
                Some("binevent") => {
                    self.mode = OutputMode::BinEvent;
                    let _ = response.push_str("OK: mode=binevent\r\n");
                }
                _ => {
                    let _ = response.push_str("ERR: out <log|event|binevent>\r\n");
                }
            },
            Some("status") => {
                let mode_str = match self.mode {
                    OutputMode::Log => "log",
                    OutputMode::Event => "event",
                    OutputMode::BinEvent => "binevent",
                };
                let fill_pct = (self.fill_level() * 100.0) as u32;
                let _ = write!(response, "mode={} fill={}%\r\n", mode_str, fill_pct);
            }
            Some("help") => {
                let _ = response.push_str("out status esc\r\n");
            }
            Some("esc") => {
                let arg2 = parts.next();
                match arg {
                    Some("raw") => {
                        // esc raw <left> <right>
                        if let (Some(l_str), Some(r_str)) = (arg2, parts.next()) {
                            if let (Ok(l), Ok(r)) = (l_str.parse::<u16>(), r_str.parse::<u16>()) {
                                let l = l.min(2047);
                                let r = r.min(2047);
                                set_esc_mode(EscMode::Override(l, r));
                                let _ = write!(response, "OK: esc raw {} {}\r\n", l, r);
                            } else {
                                let _ = response.push_str("ERR: invalid numbers\r\n");
                            }
                        } else {
                            let _ = response.push_str("ERR: esc raw <left> <right>\r\n");
                        }
                    }
                    Some("p") => {
                        // esc p <left%> <right%>  (-100 to 100 -> 0 to 2047)
                        if let (Some(l_str), Some(r_str)) = (arg2, parts.next()) {
                            if let (Ok(l), Ok(r)) = (l_str.parse::<i16>(), r_str.parse::<i16>()) {
                                let l_raw = percent_to_raw(l);
                                let r_raw = percent_to_raw(r);
                                set_esc_mode(EscMode::Override(l_raw, r_raw));
                                let _ = write!(response, "OK: esc p {}% {}% (raw {} {})\r\n", l, r, l_raw, r_raw);
                            } else {
                                let _ = response.push_str("ERR: invalid numbers\r\n");
                            }
                        } else {
                            let _ = response.push_str("ERR: esc p <left%> <right%>\r\n");
                        }
                    }
                    Some("off") => {
                        set_esc_mode(EscMode::Off);
                        let _ = response.push_str("OK: esc off (no DShot frames)\r\n");
                    }
                    Some("normal") | Some("n") => {
                        set_esc_mode(EscMode::Normal);
                        let _ = response.push_str("OK: esc normal\r\n");
                    }
                    _ => {
                        let _ = response.push_str("ERR: esc <raw|p|off|normal|n>\r\n");
                    }
                }
            }
            _ => {
                let _ = response.push_str("ERR: unknown command (try 'help')\r\n");
            }
        }
        response
    }
}

/// Convert percent (-100 to 100) to raw DShot value (0 to 2047)
/// -100% -> 0, 0% -> 1024, 100% -> 2047
fn percent_to_raw(percent: i16) -> u16 {
    let clamped = percent.clamp(-100, 100) as i32;
    // Map -100..100 to 0..2047
    // -100 -> 0, 0 -> 1024 (approx), 100 -> 2047
    let raw = ((clamped + 100) * 2047) / 200;
    raw as u16
}

/// Format event as human-readable JSON-like array
fn format_event_human(event: &InputEvent, buf: &mut heapless::Vec<u8, 256>) {
    // Use a temporary string for formatting
    let mut s: heapless::String<256> = heapless::String::new();

    match event {
        InputEvent::Lidar(ts, samples) => {
            let _ = write!(s, "[\"Lidar\",{}", ts);
            for sample in samples {
                let _ = write!(s, ",{:.0}", sample);
            }
            let _ = s.push(']');
        }
        InputEvent::Accelerometer(ts, ay, az) => {
            let _ = write!(s, "[\"Accel\",{},{:.2},{:.2}]", ts, ay, az);
        }
        InputEvent::Receiver(ts, ch, pulse) => match pulse {
            Some(p) => {
                let _ = write!(s, "[\"Receiver\",{},{},{:.1}]", ts, ch, p);
            }
            None => {
                let _ = write!(s, "[\"Receiver\",{},{},null]", ts, ch);
            }
        },
        InputEvent::Vbat(ts, voltage) => {
            let _ = write!(s, "[\"Vbat\",{},{:.2}]", ts, voltage);
        }
        InputEvent::WifiControl(ts, mode, r, m, t) => {
            let _ = write!(
                s,
                "[\"WifiControl\",{},{},{:.2},{:.2},{:.2}]",
                ts, mode, r, m, t
            );
        }
        InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) => {
            let _ = write!(
                s,
                "[\"Planner\",{},{:.1},{:.2},{:.2},{:.0},{:.0},{:.0},{:.0},{:.0},{:.0},{:.3},{:.0}]",
                ts,
                plan.rotation_speed,
                plan.movement_x,
                plan.movement_y,
                cw.0,
                cw.1,
                os.0,
                os.1,
                op.0,
                op.1,
                theta,
                rpm
            );
        }
        InputEvent::Stats(ts, stats) => {
            let _ = write!(
                s,
                "[\"Stats\",{},{},{},{}]",
                ts,
                stats.step_min_duration_us,
                stats.step_max_duration_us,
                stats.step_avg_duration_us
            );
        }
    }

    buf.extend_from_slice(s.as_bytes()).ok();
}

/// Binary event serialization (no_std compatible version)
/// Format: 2-byte tag (XOR 0x5555) + payload (little-endian)
const TAG_XOR: u16 = 0x5555;

fn serialize_event(event: &InputEvent, buf: &mut heapless::Vec<u8, 128>) {
    match event {
        InputEvent::Lidar(ts, samples) => {
            let tag = (0u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            for &s in samples {
                let _ = buf.extend_from_slice(&s.to_le_bytes());
            }
        }
        InputEvent::Accelerometer(ts, accel_y, accel_z) => {
            let tag = (1u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            let _ = buf.extend_from_slice(&accel_y.to_le_bytes());
            let _ = buf.extend_from_slice(&accel_z.to_le_bytes());
        }
        InputEvent::Receiver(ts, channel, pulse_length) => {
            let tag = (2u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            let _ = buf.push(*channel);
            match pulse_length {
                Some(pl) => {
                    let _ = buf.push(1);
                    let _ = buf.extend_from_slice(&pl.to_le_bytes());
                }
                None => {
                    let _ = buf.push(0);
                }
            }
        }
        InputEvent::Vbat(ts, voltage) => {
            let tag = (3u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            let _ = buf.extend_from_slice(&voltage.to_le_bytes());
        }
        InputEvent::WifiControl(ts, mode, r, m, t) => {
            let tag = (4u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            let _ = buf.push(*mode);
            let _ = buf.extend_from_slice(&r.to_le_bytes());
            let _ = buf.extend_from_slice(&m.to_le_bytes());
            let _ = buf.extend_from_slice(&t.to_le_bytes());
        }
        InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) => {
            let tag = (5u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            let _ = buf.extend_from_slice(&plan.rotation_speed.to_le_bytes());
            let _ = buf.extend_from_slice(&plan.movement_x.to_le_bytes());
            let _ = buf.extend_from_slice(&plan.movement_y.to_le_bytes());
            let _ = buf.extend_from_slice(&cw.0.to_le_bytes());
            let _ = buf.extend_from_slice(&cw.1.to_le_bytes());
            let _ = buf.extend_from_slice(&os.0.to_le_bytes());
            let _ = buf.extend_from_slice(&os.1.to_le_bytes());
            let _ = buf.extend_from_slice(&op.0.to_le_bytes());
            let _ = buf.extend_from_slice(&op.1.to_le_bytes());
            let _ = buf.extend_from_slice(&theta.to_le_bytes());
            let _ = buf.extend_from_slice(&rpm.to_le_bytes());
        }
        InputEvent::Stats(ts, stats) => {
            let tag = (6u16 ^ TAG_XOR).to_le_bytes();
            let _ = buf.extend_from_slice(&tag);
            let _ = buf.extend_from_slice(&ts.to_le_bytes());
            let _ = buf.extend_from_slice(&stats.step_min_duration_us.to_le_bytes());
            let _ = buf.extend_from_slice(&stats.step_max_duration_us.to_le_bytes());
            let _ = buf.extend_from_slice(&stats.step_avg_duration_us.to_le_bytes());
        }
    }
}

// Global console instances
pub type ConsoleMutex = Mutex<RefCell<ConsoleState>>;

static UART_CONSOLE: StaticCell<ConsoleMutex> = StaticCell::new();
static USB_CONSOLE: StaticCell<ConsoleMutex> = StaticCell::new();

// Global ESC mode for motor control override
static ESC_MODE: Mutex<RefCell<EscMode>> = Mutex::new(RefCell::new(EscMode::Normal));

// Store references after initialization for later retrieval
static mut UART_CONSOLE_REF: Option<&'static ConsoleMutex> = None;
static mut USB_CONSOLE_REF: Option<&'static ConsoleMutex> = None;

/// Initialize both console instances. Must be called once at startup.
pub fn init_consoles() -> (&'static ConsoleMutex, &'static ConsoleMutex) {
    let uart = UART_CONSOLE.init(Mutex::new(RefCell::new(ConsoleState::new())));
    let usb = USB_CONSOLE.init(Mutex::new(RefCell::new(ConsoleState::new())));

    // Store references for later retrieval
    unsafe {
        UART_CONSOLE_REF = Some(uart);
        USB_CONSOLE_REF = Some(usb);
    }

    (uart, usb)
}

/// Get the UART console instance. Only valid after init_consoles().
pub fn get_uart_console() -> &'static ConsoleMutex {
    unsafe { UART_CONSOLE_REF.unwrap_unchecked() }
}

/// Get the USB console instance. Only valid after init_consoles().
pub fn get_usb_console() -> &'static ConsoleMutex {
    unsafe { USB_CONSOLE_REF.unwrap_unchecked() }
}

/// Get the current ESC mode
pub fn get_esc_mode() -> EscMode {
    cortex_m::interrupt::free(|cs| *ESC_MODE.borrow(cs).borrow())
}

/// Set the ESC mode
pub fn set_esc_mode(mode: EscMode) {
    cortex_m::interrupt::free(|cs| {
        *ESC_MODE.borrow(cs).borrow_mut() = mode;
    });
}
