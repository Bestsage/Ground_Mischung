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
    // Sync byte 0xC8 for standard CRSF frames (RC channels)
    buf.push(CRSF_SYNC_BYTE).ok();
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

    // Payload: Altitude (dm), Vertical Speed (m/s)
    // Altitude in dm (decimeters)
    let alt_dm = (alt * 10.0) as u16;
    let vspd_cms = (vspeed * 100.0) as i16; // cm/s

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

    frame.push(CRSF_ADDRESS_TX).ok(); // Address: TX module
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

    frame.push(CRSF_ADDRESS_TX).ok(); // Address: TX module
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

    frame.push(CRSF_ADDRESS_TX).ok(); // Address: TX module
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
/// ELRS 3.x parameter numbers (may vary by firmware version):
/// 1=Packet Rate, 2=Telem Ratio, 3=Switch Mode, 4=Antenna Mode,
/// 5=TX Power (folder), 6=Max Power, 7=Dynamic
pub fn build_elrs_config_sequence(rate_idx: u8, tlm_idx: u8, power_idx: u8) -> [Vec<u8, 64>; 4] {
    [
        // 1. Ping TX module first
        build_device_ping(CRSF_ADDRESS_TX),
        // 2. Write packet rate (param 1 in ELRS)
        build_parameter_write(1, rate_idx),
        // 3. Write telemetry ratio (param 2 in ELRS)
        build_parameter_write(2, tlm_idx),
        // 4. Write power (param 6 = Max Power in ELRS 3.x)
        build_parameter_write(6, power_idx),
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

/// Helper: Build single config frame for TX power (Max Power = param 6)
pub fn build_elrs_tx_power(power_idx: u8) -> Vec<u8, 64> {
    build_parameter_write(6, power_idx)
}

// ============================================================================
// ELRS Parameter Discovery
// Reads parameters from the ELRS TX module to discover available power levels,
// packet rates, and telemetry ratios dynamically.
// ============================================================================

/// CRSF Parameter data types
pub const CRSF_PARAM_TYPE_UINT8: u8 = 0;
pub const CRSF_PARAM_TYPE_INT8: u8 = 1;
pub const CRSF_PARAM_TYPE_UINT16: u8 = 2;
pub const CRSF_PARAM_TYPE_INT16: u8 = 3;
pub const CRSF_PARAM_TYPE_TEXT_SELECTION: u8 = 9;
pub const CRSF_PARAM_TYPE_STRING: u8 = 10;
pub const CRSF_PARAM_TYPE_FOLDER: u8 = 11;
pub const CRSF_PARAM_TYPE_INFO: u8 = 12;
pub const CRSF_PARAM_TYPE_COMMAND: u8 = 13;

/// Discovered ELRS module configuration
/// Populated via CRSF parameter discovery at boot.
/// All fields are primitives so this is Copy-safe for use in Mutex<RefCell<>>.
#[derive(Clone, Copy)]
pub struct ElrsConfig {
    pub device_found: bool,
    pub discovery_done: bool,
    pub total_params: u8,
    // Power
    pub power_param_num: u8,           // CRSF param number for Max Power
    pub power_levels_mw: [u16; 12],    // mW values for each index
    pub power_count: u8,               // number of available power levels
    pub power_current_idx: u8,         // current power index reported by module
    // Rate
    pub rate_param_num: u8,
    pub rate_count: u8,
    pub rate_current_idx: u8,
    // TLM
    pub tlm_param_num: u8,
    pub tlm_count: u8,
    pub tlm_current_idx: u8,
}

impl Default for ElrsConfig {
    fn default() -> Self {
        Self {
            device_found: false,
            discovery_done: false,
            total_params: 0,
            power_param_num: 0,
            power_levels_mw: [0; 12],
            power_count: 0,
            power_current_idx: 0,
            rate_param_num: 0,
            rate_count: 0,
            rate_current_idx: 0,
            tlm_param_num: 0,
            tlm_count: 0,
            tlm_current_idx: 0,
        }
    }
}

/// Parse DeviceInfo (0x29) response to extract field count
/// Payload (after type byte): [Dest] [Origin] [DeviceName\0] [Serial(4)] [HW(4)] [SW(4)] [FieldCount] [ParamVer]
/// Returns field_count or None
pub fn parse_device_info(payload: &[u8]) -> Option<u8> {
    if payload.len() < 4 {
        return None;
    }
    // Device name starts at [2], null-terminated
    let name_end = payload[2..].iter().position(|&b| b == 0)?;
    let after_name = 2 + name_end + 1;
    // Skip serial(4) + hw_ver(4) + sw_ver(4) = 12 bytes
    let field_count_idx = after_name + 12;
    if field_count_idx < payload.len() {
        Some(payload[field_count_idx])
    } else {
        None
    }
}

/// Parsed result from a ParameterSettingsEntry (0x2B) frame
pub struct ParsedParam {
    pub param_num: u8,
    pub parent: u8,
    pub data_type: u8,
    pub value: u8,
    pub max: u8,
    pub is_power: bool,         // name contains "Power" (not "Dynamic")
    pub is_rate: bool,          // name contains "Rate" or "Packet"
    pub is_tlm: bool,           // name contains "Telem" or "TLM"
    pub has_mw: bool,           // options contain "mW" or "W"
    pub power_levels: [u16; 12],
    pub power_count: u8,
    pub option_count: u8,
}

/// Parse a ParameterSettingsEntry (0x2B) payload
/// Payload: [Dest] [Origin] [ParamNum] [ChunkIdx] [Parent] [DataType] [Name\0] [data...]
/// For TEXT_SELECTION: [Options\0] [Value] [Min] [Max] [Default]
pub fn parse_parameter_entry(payload: &[u8]) -> Option<ParsedParam> {
    if payload.len() < 7 {
        return None;
    }

    let param_num = payload[2];
    let chunk_idx = payload[3];
    let parent = payload[4];
    let data_type = payload[5] & 0x7F; // mask out hidden bit

    // Only handle chunk 0 (first/only chunk)
    if chunk_idx != 0 {
        return None;
    }

    // Parse name (null-terminated at index 6+)
    let name_start = 6;
    let name_end_rel = payload[name_start..].iter().position(|&b| b == 0)?;
    let name_bytes = &payload[name_start..name_start + name_end_rel];

    // Identify parameter by name
    let mut is_power = false;
    let mut is_rate = false;
    let mut is_tlm = false;

    if let Ok(name_str) = core::str::from_utf8(name_bytes) {
        // "Max Power" or "Power" but not "Dynamic Power"
        if (name_str.contains("Power") || name_str.contains("power"))
            && !name_str.contains("Dynamic")
            && !name_str.contains("dynamic")
        {
            is_power = true;
        }
        if name_str.contains("Rate") || name_str.contains("rate") || name_str.contains("Packet") {
            is_rate = true;
        }
        if name_str.contains("Telem") || name_str.contains("telem") || name_str.contains("TLM") {
            is_tlm = true;
        }
    }

    let after_name = name_start + name_end_rel + 1;

    let mut result = ParsedParam {
        param_num,
        parent,
        data_type,
        value: 0,
        max: 0,
        is_power,
        is_rate,
        is_tlm,
        has_mw: false,
        power_levels: [0; 12],
        power_count: 0,
        option_count: 0,
    };

    // Parse TEXT_SELECTION type
    if data_type == CRSF_PARAM_TYPE_TEXT_SELECTION && after_name < payload.len() {
        if let Some(opts_end_rel) = payload[after_name..].iter().position(|&b| b == 0) {
            let opts_bytes = &payload[after_name..after_name + opts_end_rel];
            if let Ok(opts_str) = core::str::from_utf8(opts_bytes) {
                result.has_mw = opts_str.contains("mW") || opts_str.contains(" W");

                // Count options and parse mW values
                let mut count: u8 = 0;
                for opt in opts_str.split(';') {
                    if count < 12 {
                        if result.has_mw {
                            if let Some(val) = parse_mw_value(opt) {
                                result.power_levels[count as usize] = val;
                                result.power_count = count + 1;
                            }
                        }
                        count += 1;
                    }
                }
                result.option_count = count;
            }

            // Value, Min, Max, Default after options\0
            let val_start = after_name + opts_end_rel + 1;
            if val_start < payload.len() {
                result.value = payload[val_start];
            }
            if val_start + 2 < payload.len() {
                result.max = payload[val_start + 2];
            }
        }
    }

    Some(result)
}

/// Parse a milliwatt value from an option string like "10 mW", "100mW", "1 W", "2000 mW"
fn parse_mw_value(s: &str) -> Option<u16> {
    let s = s.trim();
    let bytes = s.as_bytes();

    // Extract leading digits
    let mut num: u16 = 0;
    let mut found_digit = false;

    for &b in bytes {
        if b >= b'0' && b <= b'9' {
            found_digit = true;
            num = num.checked_mul(10)?.checked_add((b - b'0') as u16)?;
        } else if found_digit {
            break; // stop at first non-digit after digits
        }
    }

    if !found_digit {
        return None;
    }

    // If it says "W" but NOT "mW", value is in Watts → convert to mW
    if s.contains('W') && !s.contains("mW") {
        num = num.checked_mul(1000)?;
    }

    Some(num)
}
