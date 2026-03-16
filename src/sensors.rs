// TFA300 LIDAR Protocol Constants
pub const PACKET_SIZE: usize = 9;         // 9 bytes for TFA300 (was 22 for LDS02RR)
pub const HEAD_BYTE_1: u8 = 0x59;         // First header byte
pub const HEAD_BYTE_2: u8 = 0x59;         // Second header byte
pub const LIDAR_DISTANCE_OFFSET: f32 = 36.0; // Distance offset based on mounting position (may need retuning)

// Note: No LIDAR_ENCODER_HZ needed! TFA300 doesn't require encoder signal (major simplification)

pub struct ParsedPacket {
    pub distances: [u16; 4], // mm (TFA300 gives 1 value, replicated 4x for compatibility)
}

/// Parse TFA300 9-byte UART packet
/// Format: [0x59, 0x59, DIST_L, DIST_H, STRENGTH_L, STRENGTH_H, 0x00, 0x00, CHECKSUM]
pub fn parse_packet(packet: &[u8]) -> Option<ParsedPacket> {
    if packet.len() != PACKET_SIZE
        || packet[0] != HEAD_BYTE_1
        || packet[1] != HEAD_BYTE_2 {
        return None;
    }

    // Distance: little-endian u16 (mm)
    let distance = (packet[3] as u16) << 8 | (packet[2] as u16);

    // Signal strength: little-endian u16
    let _strength = (packet[5] as u16) << 8 | (packet[4] as u16);

    // Checksum: sum of bytes 0-7, masked to lower 8 bits
    let checksum = (packet[0..8].iter().map(|&b| b as u16).sum::<u16>() & 0xFF) as u8;
    if checksum != packet[8] {
        return None;
    }

    // TFA300 gives single distance per packet (unlike LDS02RR's 4)
    // Replicate it 4 times to maintain compatibility with existing algorithm
    Some(ParsedPacket {
        distances: [distance, distance, distance, distance],
    })
}

/// Compute TFA300 checksum (simple sum of bytes, lower 8 bits)
pub fn compute_checksum(data: &[u8]) -> u8 {
    (data.iter().map(|&b| b as u16).sum::<u16>() & 0xFF) as u8
}

// TODO: Sensor tasks will be implemented once we have pinout info:
// - lidar_publisher: async UART RX with DMA for TFA300
// - accelerometer_task: SPI for ADXL373 (same protocol, different HAL)
// - rc_receiver_task: Timer input capture for PWM measurement
