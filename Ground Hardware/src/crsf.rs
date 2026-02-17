#![allow(unused)]

use heapless::Vec;

#[derive(Debug, Clone, Copy)]
pub enum PacketType {
    Gps = 0x02,
    Vario = 0x09, // User specified 0x09
    BatterySensor = 0x08,
    Barometer = 0x11, // User specified 0x11
    LinkStatistics = 0x14,
    OpenTxSync = 0x10,
    RadioId = 0x3A,
    RcChannelsPacked = 0x16,
    Attitude = 0x1E,
    FlightMode = 0x21,
    DevicePing = 0x28,
    DeviceInfo = 0x29,
    ParameterSettingsEntry = 0x2B,
    ParameterRead = 0x2C,
    ParameterWrite = 0x2D,
    Command = 0x32,
    KissRequest = 0x78,
    KissResponse = 0x79,
    MspRequest = 0x7A,
    MspResponse = 0x7B,
    MspWrite = 0x7C,
    ArduPilotResponse = 0x80,
}

impl PacketType {
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x02 => Some(PacketType::Gps),
            0x09 => Some(PacketType::Vario),
            0x08 => Some(PacketType::BatterySensor),
            0x11 => Some(PacketType::Barometer),
            0x14 => Some(PacketType::LinkStatistics),
            0x10 => Some(PacketType::OpenTxSync),
            0x3A => Some(PacketType::RadioId),
            0x16 => Some(PacketType::RcChannelsPacked),
            0x1E => Some(PacketType::Attitude),
            0x21 => Some(PacketType::FlightMode),
            0x28 => Some(PacketType::DevicePing),
            0x29 => Some(PacketType::DeviceInfo),
            0x2B => Some(PacketType::ParameterSettingsEntry),
            0x2C => Some(PacketType::ParameterRead),
            0x2D => Some(PacketType::ParameterWrite),
            0x32 => Some(PacketType::Command),
            0x78 => Some(PacketType::KissRequest),
            0x79 => Some(PacketType::KissResponse),
            0x7A => Some(PacketType::MspRequest),
            0x7B => Some(PacketType::MspResponse),
            0x7C => Some(PacketType::MspWrite),
            0x80 => Some(PacketType::ArduPilotResponse),
            _ => None,
        }
    }
}

pub struct CrsfParser {
    state: ParserState,
    buffer: Vec<u8, 64>, // Max packet size 64 as per spec (usually)
    packet_len: u8,
    packet_type: u8,
}

enum ParserState {
    Sync,
    Length,
    Type,
    Payload,
    Crc,
}

impl CrsfParser {
    pub fn new() -> Self {
        Self {
            state: ParserState::Sync,
            buffer: Vec::new(),
            packet_len: 0,
            packet_type: 0,
        }
    }

    pub fn push_byte(&mut self, byte: u8) -> Option<(PacketType, Vec<u8, 64>)> {
        match self.state {
            ParserState::Sync => {
                if byte == 0xC8 || byte == 0xEA || byte == 0xEE || byte == 0xEC {
                    self.buffer.clear();
                    // self.buffer.push(byte).ok(); We don't need sync byte in payload usually, but for CRC we might need checks.
                    // CRSF CRC includes Type + Payload. Sync and Len are excluded.
                    // But wait, CRC calc usually covers Type + Payload.
                    self.state = ParserState::Length;
                }
            }
            ParserState::Length => {
                if byte > 62 || byte < 2 {
                    self.state = ParserState::Sync;
                } else {
                    self.packet_len = byte;
                    self.state = ParserState::Type;
                }
            }
            ParserState::Type => {
                self.packet_type = byte;
                self.buffer.push(byte).ok();
                self.state = ParserState::Payload;
            }
            ParserState::Payload => {
                self.buffer.push(byte).ok();
                // Length byte includes Type + Payload + CRC
                // So Payload len = Length - 2 (Type + CRC)
                // We already pushed Type.
                // Current buffer len = 1 (Type) + Payload bytes
                // Target buffer len = Length - 1 (since CRC is last)
                if self.buffer.len() as u8 == self.packet_len - 1 {
                    self.state = ParserState::Crc;
                }
            }
            ParserState::Crc => {
                self.state = ParserState::Sync;
                let crc_calculated = crc8(&self.buffer);
                if crc_calculated == byte {
                    // Valid packet
                    // Extract payload (remove Type from start)
                    let ptype = PacketType::from_byte(self.packet_type)?;
                    // Clone payload excluding type
                    let mut payload = Vec::new();
                    // self.buffer[0] is Type
                    payload.extend_from_slice(&self.buffer[1..]).ok();
                    return Some((ptype, payload));
                }
            }
        }
        None
    }

    pub fn push_bytes(&mut self, bytes: &[u8]) -> Option<(PacketType, Vec<u8, 64>)> {
        for b in bytes {
            if let Some(packet) = self.push_byte(*b) {
                return Some(packet);
            }
        }
        None
    }
}

fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// Building packets
// Channels: 0x16. 16 channels, 11 bits each independent of RC frame.
// Packed into 22 bytes.

#[derive(Debug, Clone)]
pub struct RcChannels {
    pub channels: [u16; 16],
}

/// RadioId packet - sent by ELRS for timing sync and radio identification
/// Payload structure (10 bytes after type):
///   - byte 0: sub-type (0x10 = timing sync)
///   - bytes 1-4: timing offset (u32, little-endian, microseconds)
///   - bytes 5-8: radio UID or interval data
///   - byte 9: flags/phase
#[derive(Debug, Clone, Copy, Default)]
pub struct RadioId {
    pub sub_type: u8,
    pub timing_offset_us: u32,
    pub interval_us: u32,
    pub phase: u8,
}

impl RadioId {
    /// Parse RadioId from payload (excluding type byte)
    /// Returns None if payload is too short
    pub fn from_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() < 10 {
            return None;
        }

        let sub_type = payload[0];

        // Timing offset: 4 bytes little-endian
        let timing_offset_us = u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);

        // Interval/UID: 4 bytes little-endian
        let interval_us = u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);

        let phase = payload[9];

        Some(Self {
            sub_type,
            timing_offset_us,
            interval_us,
            phase,
        })
    }
}

pub fn build_channels_packet(channels: &[u16; 16]) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    // Sync
    buf.push(0xC8).ok(); // Sync byte for RC channels
                         // Len: 22 bytes payload + 1 type + 1 crc = 24
    buf.push(24).ok();
    // Type
    buf.push(0x16).ok(); // RcChannelsPacked

    // Default 0
    let mut payload = [0u8; 22];

    // Packing 11 bits
    // Simplified packing loop or logic
    // Mask Ch1 11 bits to 0..10
    // bits are packed little endian stream?
    // standard CRSF packing:
    //  byte 0: ch0[0-7]
    //  byte 1: ch0[8-10] | ch1[0-4] << 3
    //  etc.
    // For now, let's implement the standard C-style packing ported to Rust

    let mut ch_bits: u16;
    let mut info_bits_remaining: usize = 11;
    let mut payload_bit_idx: usize = 0;

    // Not strictly efficient but functional bit blasting
    for ch in channels {
        let mut val = *ch;
        for _ in 0..11 {
            let bit = (val & 1) as u8;
            val >>= 1;

            let byte_idx = payload_bit_idx / 8;
            let bit_offset = payload_bit_idx % 8;
            payload[byte_idx] |= bit << bit_offset;

            payload_bit_idx += 1;
        }
    }

    buf.extend_from_slice(&payload).ok();

    // CRC over Type + Payload
    // Type is at index 2
    let crc = crc8(&buf[2..]);
    buf.push(crc).ok();

    buf
}

// Telemetry Packets building helpers (mostly for Flight Controller side, but we might simulate)
pub const CRSF_FRAMETYPE_GPS: u8 = 0x02;
pub const CRSF_FRAMETYPE_BATTERY_SENSOR: u8 = 0x08;
pub const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;

pub fn build_telemetry_packet(buf: &[u8]) -> Vec<u8, 64> {
    // Helper generic wrapper
    let mut packet = Vec::new();
    // Sync
    packet.push(0xC8).ok();
    // Len = Payload len + Type(1) + Crc(1).
    // Input buf includes Type as first byte? No, usually separate.
    // Assume buf is (Type + Payload).
    packet.push((buf.len() + 1) as u8).ok();
    packet.extend_from_slice(buf).ok();
    let crc = crc8(buf);
    packet.push(crc).ok();
    packet
}

// GPS Payload Construction
// Payload:
// int32 lat (deg * 1e7)
// int32 lon (deg * 1e7)
// uint16 speed (km/h * 10)
// uint16 heading (deg * 100)
// uint16 alt (m + 1000)
// uint8 sat
pub fn payload_gps(
    lat: f32,
    lon: f32,
    speed: f32,
    heading: f32,
    alt: f32,
    sats: u8,
) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    buf.push(CRSF_FRAMETYPE_GPS).ok();

    let lat_i = (lat * 10_000_000.0) as i32;
    let lon_i = (lon * 10_000_000.0) as i32;
    let spd_i = (speed * 10.0) as u16; // km/h
    let hdg_i = (heading * 100.0) as u16;
    let alt_i = (alt + 1000.0) as u16;

    buf.extend_from_slice(&lat_i.to_be_bytes()).ok();
    buf.extend_from_slice(&lon_i.to_be_bytes()).ok();
    buf.extend_from_slice(&spd_i.to_be_bytes()).ok();
    buf.extend_from_slice(&hdg_i.to_be_bytes()).ok();
    buf.extend_from_slice(&alt_i.to_be_bytes()).ok();
    buf.push(sats).ok();

    buf
}

pub fn payload_attitude(pitch: f32, roll: f32, yaw: f32) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    buf.push(CRSF_FRAMETYPE_ATTITUDE).ok();
    // int16 pitch (rad * 10000)
    // int16 roll (rad * 10000)
    // int16 yaw (rad * 10000)
    let p = (pitch * 10000.0) as i16;
    let r = (roll * 10000.0) as i16;
    let y = (yaw * 10000.0) as i16;

    buf.extend_from_slice(&p.to_be_bytes()).ok();
    buf.extend_from_slice(&r.to_be_bytes()).ok();
    buf.extend_from_slice(&y.to_be_bytes()).ok();
    buf
}

pub fn payload_battery(voltage: f32, current: f32, fuel: f32, remaining: u8) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    buf.push(CRSF_FRAMETYPE_BATTERY_SENSOR).ok();
    // uint16 voltage (dV * 10) => V * 10
    // uint16 current (dA * 10) => A * 10
    // uint24 fuel (mAh)
    // uint8 remaining (%)

    let v = (voltage * 10.0) as u16;
    let c = (current * 10.0) as u16;
    let f = fuel as u32; // mask to 24 bits

    buf.extend_from_slice(&v.to_be_bytes()).ok();
    buf.extend_from_slice(&c.to_be_bytes()).ok();

    // 24 bit Fuel - Big Endian
    buf.push(((f >> 16) & 0xFF) as u8).ok();
    buf.push(((f >> 8) & 0xFF) as u8).ok();
    buf.push((f & 0xFF) as u8).ok();

    buf.push(remaining).ok();
    buf
}

// Flight Mode: 0x21
pub const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;

// User specified Vario ID 0x09
pub const CRSF_FRAMETYPE_VARIO: u8 = 0x09;
pub const CRSF_FRAMETYPE_BAROMETER: u8 = 0x11;

pub fn payload_vario(alt: f32, vspeed: f32) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    buf.push(CRSF_FRAMETYPE_VARIO).ok();

    // Payload: Altitude (dm), Vertical Speed (cm/s)
    // Altitude in dm (decimeters)
    let alt_dm = (alt * 10.0) as u16;
    let vspd_cms = (vspeed * 100.0) as i16;

    buf.extend_from_slice(&alt_dm.to_be_bytes()).ok();
    buf.extend_from_slice(&vspd_cms.to_be_bytes()).ok();

    buf
}

pub fn payload_barometer(pressure_pa: f32, temp_c: f32) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    buf.push(CRSF_FRAMETYPE_BAROMETER).ok();

    // Pressure (Pa - Big Endian u32? or u16? Pa can be 100000. u16 max is 65535. Must be u32 or scaled)
    // Standard OpenTx/ELRS barometer:
    // Some implementations: Pressure (hPa/mbar * 10 or Pa?)
    // User request: "Pressure (Pa), Temp (0.01 C)"
    // Looking at common docs:
    // Baro (0x11 or similar?): usually not standard CRSF, but supported by ELRS/EdgeTX?
    // Let's assume u32 for Pa and i16 for Temp (0.01 deg)

    let press = pressure_pa as u32; // Pa
    let temp = (temp_c * 100.0) as i16; // 0.01 C

    // Usually Big Endian
    buf.extend_from_slice(&press.to_be_bytes()).ok();
    buf.extend_from_slice(&temp.to_be_bytes()).ok();

    buf
}

pub fn payload_flight_mode(mode: &str) -> Vec<u8, 64> {
    let mut buf = Vec::new();
    buf.push(CRSF_FRAMETYPE_FLIGHT_MODE).ok();

    // String + null terminator
    buf.extend_from_slice(mode.as_bytes()).ok();
    buf.push(0).ok();

    buf
}

// ============================================================================
// ELRS Configuration via CRSF Protocol
// Based on official CRSF specification
// ============================================================================

// CRSF Device Addresses (from spec)
pub const CRSF_SYNC_BYTE: u8 = 0xC8;
pub const CRSF_ADDRESS_BROADCAST: u8 = 0x00;
pub const CRSF_ADDRESS_FC: u8 = 0xC8; // Flight Controller
pub const CRSF_ADDRESS_HANDSET: u8 = 0xEA; // Remote Control (us)
pub const CRSF_ADDRESS_RX: u8 = 0xEC; // R/C Receiver
pub const CRSF_ADDRESS_TX: u8 = 0xEE; // R/C Transmitter Module

// CRSF Frame Types (Extended Header)
pub const CRSF_FRAMETYPE_DEVICE_PING: u8 = 0x28;
pub const CRSF_FRAMETYPE_DEVICE_INFO: u8 = 0x29;
pub const CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: u8 = 0x2B;
pub const CRSF_FRAMETYPE_PARAMETER_READ: u8 = 0x2C;
pub const CRSF_FRAMETYPE_PARAMETER_WRITE: u8 = 0x2D;

// Note: crc8() function is already defined above (line 150) using polynomial 0xD5

/// Build a Device Ping frame (0x28)
/// This should be sent first to wake up the TX module for configuration
/// Destination can be broadcast (0x00) or specific device
pub fn build_device_ping(destination: u8) -> Vec<u8, 64> {
    let mut frame = Vec::new();

    // Extended Header Frame format:
    // [Sync] [FrameLen] [Type] [Dest] [Origin] [CRC]
    // Frame length = Type(1) + Dest(1) + Origin(1) + CRC(1) = 4

    frame.push(CRSF_SYNC_BYTE).ok();
    frame.push(4).ok(); // Frame length
    frame.push(CRSF_FRAMETYPE_DEVICE_PING).ok();
    frame.push(destination).ok();
    frame.push(CRSF_ADDRESS_HANDSET).ok(); // We are the handset/RC

    // CRC covers Type + Dest + Origin (bytes 2..5)
    let crc = crc8(&frame[2..]);
    frame.push(crc).ok();

    frame
}

/// Build a Parameter Read request frame (0x2C)
/// Used to request parameter info from a device
pub fn build_parameter_read(param_number: u8, chunk: u8) -> Vec<u8, 64> {
    let mut frame = Vec::new();

    // Extended Header Frame with payload:
    // [Sync] [FrameLen] [Type] [Dest] [Origin] [ParamNum] [Chunk] [CRC]
    // Frame length = Type(1) + Dest(1) + Origin(1) + Payload(2) + CRC(1) = 6

    frame.push(CRSF_SYNC_BYTE).ok();
    frame.push(6).ok();
    frame.push(CRSF_FRAMETYPE_PARAMETER_READ).ok();
    frame.push(CRSF_ADDRESS_TX).ok(); // Send to TX module
    frame.push(CRSF_ADDRESS_HANDSET).ok(); // We are handset
    frame.push(param_number).ok();
    frame.push(chunk).ok();

    let crc = crc8(&frame[2..]);
    frame.push(crc).ok();

    frame
}

/// Build a Parameter Write frame (0x2D)
/// Used to set a parameter value on the TX module
///
/// For ELRS, parameter numbers depend on firmware version, but typically:
/// - Discover parameters first using Parameter Read
/// - Then write using the correct parameter number
pub fn build_parameter_write(param_number: u8, value: u8) -> Vec<u8, 64> {
    let mut frame = Vec::new();

    // Extended Header Frame with payload:
    // [Sync] [FrameLen] [Type] [Dest] [Origin] [ParamNum] [Value] [CRC]
    // Frame length = Type(1) + Dest(1) + Origin(1) + ParamNum(1) + Value(1) + CRC(1) = 6

    frame.push(CRSF_SYNC_BYTE).ok();
    frame.push(6).ok();
    frame.push(CRSF_FRAMETYPE_PARAMETER_WRITE).ok();
    frame.push(CRSF_ADDRESS_TX).ok(); // Send to TX module
    frame.push(CRSF_ADDRESS_HANDSET).ok(); // We are handset
    frame.push(param_number).ok();
    frame.push(value).ok();

    let crc = crc8(&frame[2..]);
    frame.push(crc).ok();

    frame
}

/// Build ELRS-specific configuration sequence
/// Returns a vector of frames to send in sequence
///
/// ELRS parameter numbers (may vary by firmware version):
/// Check your ELRS Lua script for exact numbers
/// Typically found in the ELRS source: src/lua/elrsV3.lua
pub fn build_elrs_config_sequence(rate_idx: u8, tlm_idx: u8, power_idx: u8) -> [Vec<u8, 64>; 4] {
    [
        // 1. Ping TX module first
        build_device_ping(CRSF_ADDRESS_TX),
        // 2. Write packet rate (typically param 1 in ELRS)
        build_parameter_write(1, rate_idx),
        // 3. Write telemetry ratio (typically param 2 in ELRS)
        build_parameter_write(2, tlm_idx),
        // 4. Write power (typically param 3 in ELRS)
        build_parameter_write(3, power_idx),
    ]
}

/// Helper: Build single config frame for packet rate
pub fn build_elrs_packet_rate(rate_idx: u8) -> Vec<u8, 64> {
    build_parameter_write(1, rate_idx)
}

/// Helper: Build single config frame for telemetry ratio
pub fn build_elrs_tlm_ratio(tlm_idx: u8) -> Vec<u8, 64> {
    build_parameter_write(2, tlm_idx)
}

/// Helper: Build single config frame for TX power
pub fn build_elrs_tx_power(power_idx: u8) -> Vec<u8, 64> {
    build_parameter_write(3, power_idx)
}
