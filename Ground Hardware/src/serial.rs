//! Serial Protocol for PC communication
//!
//! Output format (JSON-like, easy to parse):
//! DATA:{"rssi":-40,"lq":98,"pitch":1.2,"roll":-0.3,"yaw":45.0,"vario":0.5,"batt":3700,"sats":8}
//!
//! Input commands:
//! CMD:TLM:3      -> Set telemetry ratio to index 3
//! CMD:RATE:2     -> Set packet rate to index 2
//! CMD:PWR:75     -> Set TX power to 75%
//! CMD:THEME:1    -> Set theme index to 1
//! CMD:ARM:1      -> Enable arm switch
//! CMD:ARM:0      -> Disable arm switch
//! CMD:SCREEN:0-2 -> Change screen

use heapless::String;

/// Build telemetry JSON string for PC
pub fn build_telemetry_json(
    uplink_rssi: i32,
    downlink_rssi: i32,
    link_quality: u8,
    pitch: f32,
    roll: f32,
    yaw: f32,
    vario: f32,
    batt_mv: u32,
    satellites: u8,
    alt: f32,
    packet_count: u32,
) -> String<256> {
    let mut buf = String::new();

    // Simple JSON format - easy to parse
    // Added "drssi" for Downlink RSSI
    let _ = core::fmt::write(&mut buf, format_args!(
        "DATA:{{\"rssi\":{},\"drssi\":{},\"lq\":{},\"p\":{:.1},\"r\":{:.1},\"y\":{:.1},\"vario\":{:.2},\"batt\":{},\"sats\":{},\"alt\":{:.0},\"pkts\":{}}}",
        uplink_rssi, downlink_rssi, link_quality, pitch, roll, yaw, vario, batt_mv, satellites, alt, packet_count
    ));

    buf
}

/// Build config status JSON
pub fn build_config_json(
    tlm_ratio: u8,
    packet_rate: u8,
    tx_power: u8,
    theme: u8,
    screen: u8,
    is_active: bool,
) -> String<128> {
    let mut buf = String::new();

    let _ = core::fmt::write(
        &mut buf,
        format_args!(
            "CFG:{{\"tlm\":{},\"rate\":{},\"pwr\":{},\"theme\":{},\"scr\":{},\"act\":{}}}",
            tlm_ratio, packet_rate, tx_power, theme, screen, is_active as u8
        ),
    );

    buf
}

/// Parse incoming command from PC
/// Returns (command_type, value) or None if invalid
pub fn parse_command(input: &str) -> Option<(Command, u8)> {
    // Expected format: "CMD:TYPE:VALUE\n"
    let input = input.trim();

    if !input.starts_with("CMD:") {
        return None;
    }

    let rest = &input[4..];
    let mut parts = rest.split(':');

    let cmd_type = parts.next()?;
    let value_str = parts.next()?;
    let value: u8 = value_str.parse().ok()?;

    let cmd = match cmd_type {
        "TLM" => Command::SetTlmRatio,
        "RATE" => Command::SetPacketRate,
        "PWR" => Command::SetTxPower,
        "THEME" => Command::SetTheme,
        "ARM" => Command::SetArm,
        "SCREEN" => Command::SetScreen,
        "ENTER" => Command::ToggleActive,
        "SELECT" => Command::SetSelected,
        _ => return None,
    };

    Some((cmd, value))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    SetTlmRatio,
    SetPacketRate,
    SetTxPower,
    SetTheme,
    SetArm,
    SetScreen,
    ToggleActive,
    SetSelected,
}

/// Command line buffer for accumulating incoming bytes
pub struct CommandBuffer {
    buf: [u8; 64],
    len: usize,
}

impl CommandBuffer {
    pub const fn new() -> Self {
        Self {
            buf: [0; 64],
            len: 0,
        }
    }

    /// Push a byte, return complete command if newline received
    pub fn push(&mut self, byte: u8) -> Option<&str> {
        if byte == b'\n' || byte == b'\r' {
            if self.len > 0 {
                let cmd = core::str::from_utf8(&self.buf[..self.len]).ok();
                self.len = 0;
                return cmd;
            }
        } else if self.len < self.buf.len() - 1 {
            self.buf[self.len] = byte;
            self.len += 1;
        }
        None
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}
