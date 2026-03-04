//! display.rs — Single ST7789 screen, maximum debug + flight info
//! Screen: 240×280 (ST7789, SPI, CS=GPIO13, DC=GPIO8, RST=GPIO14)
//!
//! Layout (all values update every frame, labels once at boot):
//!  [0..27]   HEADER   : status dot | UP/DN RSSI | LQ
//!  [28..93]  GPS      : LAT / LON / ALT + SATS
//!  [94..123] IMU      : Pitch / Roll / Yaw
//!  [124..153]BARO     : Pressure + Temp
//!  [154..173]VARIO    : vertical speed
//!  [174..198]BATT     : mV + armed flag
//!  [199..237]DEBUG    : packet counters, gps count, radio-id timing
//!  [238..279]FOOTER   : uptime + config flags

use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, ascii::FONT_6X13, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_bus::spi::RefCellDevice;
use esp_hal::{delay::Delay, gpio::Output, spi::master::Spi};
use heapless::String;
use mipidsi::{
    models::ST7789,
    options::{ColorInversion, Orientation},
    Builder,
};
use embedded_hal::delay::DelayNs;

use crate::RocketState;

// ─────────────────────────────────────────────────────────────────────────────
// Pin helpers
// ─────────────────────────────────────────────────────────────────────────────

/// SharedPin: allows sharing a RefCell<OutputPin> across the SPI-interface and
/// the display driver without unsafe code.
#[derive(Clone, Copy)]
pub struct SharedPin<'a, P> {
    pub pin: &'a RefCell<P>,
}
impl<'a, P: OutputPin> ErrorType for SharedPin<'a, P> {
    type Error = P::Error;
}
impl<'a, P: OutputPin> OutputPin for SharedPin<'a, P> {
    fn set_low(&mut self) -> Result<(), Self::Error> { self.pin.borrow_mut().set_low() }
    fn set_high(&mut self) -> Result<(), Self::Error> { self.pin.borrow_mut().set_high() }
}

/// Dummy reset pin — RST is toggled manually before init.
pub struct NoResetPin;
impl OutputPin for NoResetPin {
    fn set_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
    fn set_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
}
impl ErrorType for NoResetPin {
    type Error = core::convert::Infallible;
}

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

type SpiType<'d> = Spi<'d, esp_hal::Blocking>;
type DisplayType<'d> = mipidsi::Display<
    SPIInterface<RefCellDevice<'d, SpiType<'d>, Output<'d>, Delay>, SharedPin<'d, Output<'d>>>,
    ST7789,
    NoResetPin,
>;

// ─────────────────────────────────────────────────────────────────────────────
// Color palette
// ─────────────────────────────────────────────────────────────────────────────
const BG        : Rgb565 = Rgb565::new(1, 2, 2);       // near-black
const HDR_BG    : Rgb565 = Rgb565::new(0, 6, 13);      // dark blue
const FTR_BG    : Rgb565 = Rgb565::new(0, 4, 8);
const ACCENT    : Rgb565 = Rgb565::new(0, 20, 31);     // cyan
const LBL_GRAY  : Rgb565 = Rgb565::new(12, 20, 12);    // mid gray (label)
const OK_GREEN  : Rgb565 = Rgb565::new(0, 30, 7);
const WARN_YLW  : Rgb565 = Rgb565::new(31, 27, 0);
const ERR_RED   : Rgb565 = Rgb565::new(31, 4, 2);
const WHITE     : Rgb565 = Rgb565::WHITE;
const DIM_WHITE : Rgb565 = Rgb565::new(20, 40, 20);

// ─────────────────────────────────────────────────────────────────────────────
// Driver
// ─────────────────────────────────────────────────────────────────────────────

pub struct Driver<'d> {
    display: DisplayType<'d>,
    frame_count: u32,
}

impl<'d> Driver<'d> {
    /// Initialise the single display.
    /// cs  : chip-select for this screen (GPIO13)
    /// dc  : data/command pin for this screen (GPIO8)
    /// rst_bus : shared reset line (GPIO14) — toggled once here
    pub fn new(
        spi_bus : &'d RefCell<SpiType<'d>>,
        dc_bus  : &'d RefCell<Output<'d>>,
        rst_bus : &'d RefCell<Output<'d>>,
        cs      : Output<'d>,
        delay   : &mut Delay,
    ) -> Self {
        // Hardware reset
        let _ = rst_bus.borrow_mut().set_low();
        delay.delay_ms(10);
        let _ = rst_bus.borrow_mut().set_high();
        delay.delay_ms(150);

        let dc = SharedPin { pin: dc_bus };
        let device = RefCellDevice::new(spi_bus, cs, Delay::new()).unwrap();
        let di = SPIInterface::new(device, dc);

        let mut display = Builder::new(ST7789, di)
            .display_size(240, 280)
            .display_offset(0, 20)
            .orientation(Orientation::default())
            .invert_colors(ColorInversion::Inverted)
            .reset_pin(NoResetPin)
            .init(delay)
            .unwrap();

        let _ = display.clear(BG);
        Self { display, frame_count: 0 }
    }

    /// Called every frame. draw_labels=true only on frame 1 (static text).
    pub fn update(&mut self, state: &RocketState, uptime_secs: u32, armed: bool, cfg_dirty: bool) {
        self.frame_count = self.frame_count.wrapping_add(1);
        let first = self.frame_count == 1;
        if first {
            let _ = self.display.clear(BG);
        }
        Self::draw_all(&mut self.display, state, uptime_secs, armed, cfg_dirty, first);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Master draw — called every frame
    // ─────────────────────────────────────────────────────────────────────────
    fn draw_all(
        d         : &mut DisplayType<'d>,
        state     : &RocketState,
        uptime    : u32,
        armed     : bool,
        cfg_dirty : bool,
        labels    : bool,
    ) {
        let mut buf = String::<64>::new();

        // ── Inline style helpers (background = BG so values erase themselves) ──
        let val_w = MonoTextStyleBuilder::new()  // white  value
            .font(&FONT_6X13).text_color(WHITE).background_color(BG).build();
        let val_g = MonoTextStyleBuilder::new()  // green  value
            .font(&FONT_6X13).text_color(OK_GREEN).background_color(BG).build();
        let val_y = MonoTextStyleBuilder::new()  // yellow value
            .font(&FONT_6X13).text_color(WARN_YLW).background_color(BG).build();
        let val_r = MonoTextStyleBuilder::new()  // red    value
            .font(&FONT_6X13).text_color(ERR_RED).background_color(BG).build();
        let val_c = MonoTextStyleBuilder::new()  // cyan   value
            .font(&FONT_6X13).text_color(ACCENT).background_color(BG).build();
        let val_d = MonoTextStyleBuilder::new()  // dim    value
            .font(&FONT_6X10).text_color(DIM_WHITE).background_color(BG).build();
        let lbl   = MonoTextStyle::new(&FONT_6X10, LBL_GRAY);
        let acc_s = MonoTextStyle::new(&FONT_6X10, ACCENT);

        // ── Colour helpers ────────────────────────────────────────────────────
        fn rssi_color(v: i32) -> Rgb565 {
            if v > -65 { OK_GREEN } else if v > -85 { WARN_YLW } else { ERR_RED }
        }
        fn lq_color(v: u8) -> Rgb565 {
            if v > 80 { OK_GREEN } else if v > 50 { WARN_YLW } else { ERR_RED }
        }

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 0 — HEADER  (y 0..27)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Rectangle::new(Point::new(0, 0), Size::new(240, 27))
                .into_styled(PrimitiveStyle::with_fill(HDR_BG)).draw(d);
            let hdr_lbl = MonoTextStyleBuilder::new()
                .font(&FONT_6X10).text_color(ACCENT).background_color(HDR_BG).build();
            let _ = Text::new("UP:", Point::new(4, 18), hdr_lbl).draw(d);
            let _ = Text::new("DN:", Point::new(76, 18), hdr_lbl).draw(d);
            let _ = Text::new("LQ:", Point::new(148, 18), hdr_lbl).draw(d);
        }
        // Status dot
        let dot_color = lq_color(state.link_quality);
        let _ = Circle::new(Point::new(218, 4), 18)
            .into_styled(PrimitiveStyle::with_fill(dot_color)).draw(d);

        // UP RSSI
        let up_style = MonoTextStyleBuilder::new().font(&FONT_6X13)
            .text_color(rssi_color(state.uplink_rssi)).background_color(HDR_BG).build();
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}", state.uplink_rssi));
        let _ = Text::new(&buf, Point::new(22, 19), up_style).draw(d);

        // DN RSSI
        let dn_style = MonoTextStyleBuilder::new().font(&FONT_6X13)
            .text_color(rssi_color(state.downlink_rssi)).background_color(HDR_BG).build();
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}", state.downlink_rssi));
        let _ = Text::new(&buf, Point::new(94, 19), dn_style).draw(d);

        // LQ
        let lq_style = MonoTextStyleBuilder::new().font(&FONT_6X13)
            .text_color(lq_color(state.link_quality)).background_color(HDR_BG).build();
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:3}%", state.link_quality));
        let _ = Text::new(&buf, Point::new(166, 19), lq_style).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 1 — GPS  (y 28..93)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Text::new("[ GPS ]", Point::new(4, 40), acc_s).draw(d);
            let _ = Text::new("LAT", Point::new(4, 54), lbl).draw(d);
            let _ = Text::new("LON", Point::new(4, 68), lbl).draw(d);
            let _ = Text::new("ALT", Point::new(4, 82), lbl).draw(d);
            let _ = Text::new("SATS", Point::new(136, 82), lbl).draw(d);
        }
        // Satellite-count dependent color for GPS values
        let gps_style = if state.satellites >= 6 { val_g }
                        else if state.satellites >= 3 { val_y }
                        else { val_r };

        // LAT: raw i32 (deg*1e7) → display as decimal with 6 dp
        let lat_deg = state.lat / 10_000_000_i32;
        let lat_frac = (state.lat.abs() % 10_000_000) / 10; // 6 digits
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}.{:06}  ", lat_deg, lat_frac));
        let _ = Text::new(&buf, Point::new(26, 54), gps_style).draw(d);

        // LON
        let lon_deg = state.lon / 10_000_000_i32;
        let lon_frac = (state.lon.abs() % 10_000_000) / 10;
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}.{:06}  ", lon_deg, lon_frac));
        let _ = Text::new(&buf, Point::new(26, 68), gps_style).draw(d);

        // ALT
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:6.1}m  ", state.alt));
        let _ = Text::new(&buf, Point::new(26, 82), val_w).draw(d);

        // SATS
        let sat_style = if state.satellites >= 6 { val_g }
                        else if state.satellites >= 3 { val_y }
                        else { val_r };
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:2}  ", state.satellites));
        let _ = Text::new(&buf, Point::new(162, 82), sat_style).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 2 — IMU / ATTITUDE  (y 94..123)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Text::new("[ IMU ]", Point::new(4, 97), acc_s).draw(d);
            let _ = Text::new("P", Point::new(4, 115), lbl).draw(d);
            let _ = Text::new("R", Point::new(82, 115), lbl).draw(d);
            let _ = Text::new("Y", Point::new(160, 115), lbl).draw(d);
        }
        // Pitch
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:+6.1}  ", state.gyro[0]));
        let _ = Text::new(&buf, Point::new(14, 115), val_w).draw(d);
        // Roll
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:+6.1}  ", state.gyro[1]));
        let _ = Text::new(&buf, Point::new(92, 115), val_w).draw(d);
        // Yaw
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:+6.1}  ", state.gyro[2]));
        let _ = Text::new(&buf, Point::new(170, 115), val_w).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 3 — BARO  (y 124..153)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Text::new("[ BARO ]", Point::new(4, 127), acc_s).draw(d);
            let _ = Text::new("hPa", Point::new(4, 143), lbl).draw(d);
            let _ = Text::new("Tmp", Point::new(122, 143), lbl).draw(d);
        }
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:7.1}  ", state.press));
        let _ = Text::new(&buf, Point::new(28, 143), val_w).draw(d);

        let temp_style = if state.temp > 60.0 { val_r } else if state.temp > 40.0 { val_y } else { val_w };
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:+5.1}C  ", state.temp));
        let _ = Text::new(&buf, Point::new(146, 143), temp_style).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 4 — VARIO  (y 154..173)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Text::new("[ VARIO ]", Point::new(4, 157), acc_s).draw(d);
            let _ = Text::new("m/s", Point::new(100, 173), lbl).draw(d);
        }
        let vario_style = if state.vel > 2.0 { val_g } else if state.vel < -3.0 { val_r } else { val_c };
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:+7.2}  ", state.vel));
        let _ = Text::new(&buf, Point::new(4, 173), vario_style).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 5 — BATTERY  (y 174..198)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Text::new("[ BATT ]", Point::new(4, 177), acc_s).draw(d);
            let _ = Text::new("mV", Point::new(82, 193), lbl).draw(d);
        }
        // Batt color: >3.7V green, 3.5-3.7 yellow, <3.5 red
        let batt_mv = state.batt_voltage;
        let batt_style = if batt_mv > 3700 { val_g } else if batt_mv > 3500 { val_y } else { val_r };
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:5}  ", batt_mv));
        let _ = Text::new(&buf, Point::new(4, 193), batt_style).draw(d);

        // Armed flag
        let arm_style = MonoTextStyleBuilder::new().font(&FONT_6X13)
            .text_color(if armed { ERR_RED } else { DIM_WHITE })
            .background_color(BG).build();
        let _ = Text::new(if armed { "[ARMED]" } else { "[ safe]" }, Point::new(134, 193), arm_style).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 6 — DEBUG COUNTERS  (y 199..237)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Text::new("[ DEBUG ]", Point::new(4, 202), acc_s).draw(d);
            let _ = Text::new("PKT", Point::new(4, 215), lbl).draw(d);
            let _ = Text::new("LS", Point::new(80, 215), lbl).draw(d);
            let _ = Text::new("GPS", Point::new(138, 215), lbl).draw(d);
            let _ = Text::new("TIM us", Point::new(4, 231), lbl).draw(d);
            let _ = Text::new("INT us", Point::new(122, 231), lbl).draw(d);
        }
        // Counters row 1
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:5}  ", state.elrs_packet_count));
        let _ = Text::new(&buf, Point::new(22, 215), val_d).draw(d);

        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}  ", state.elrs_link_stats_count));
        let _ = Text::new(&buf, Point::new(92, 215), val_d).draw(d);

        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}  ", state.elrs_gps_count));
        let _ = Text::new(&buf, Point::new(157, 215), val_d).draw(d);

        // RadioId timing row 2
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:6}  ", state.elrs_last_radio_id.timing_offset_us));
        let _ = Text::new(&buf, Point::new(40, 231), val_d).draw(d);

        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:6}  ", state.elrs_last_radio_id.interval_us));
        let _ = Text::new(&buf, Point::new(158, 231), val_d).draw(d);

        // ══════════════════════════════════════════════════════════════════════
        // SECTION 7 — FOOTER  (y 238..279)
        // ══════════════════════════════════════════════════════════════════════
        if labels {
            let _ = Rectangle::new(Point::new(0, 238), Size::new(240, 42))
                .into_styled(PrimitiveStyle::with_fill(FTR_BG)).draw(d);
            let _ = Text::new("UP:", Point::new(4, 252), MonoTextStyle::new(&FONT_6X10, LBL_GRAY)).draw(d);
            let _ = Text::new("OTH:", Point::new(100, 252), MonoTextStyle::new(&FONT_6X10, LBL_GRAY)).draw(d);
        }

        // Uptime
        let ftr_val = MonoTextStyleBuilder::new()
            .font(&FONT_6X10).text_color(DIM_WHITE).background_color(FTR_BG).build();
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:5}s  ", uptime));
        let _ = Text::new(&buf, Point::new(20, 252), ftr_val).draw(d);

        // Other packet counter
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:4}  ", state.elrs_other_count));
        let _ = Text::new(&buf, Point::new(124, 252), ftr_val).draw(d);

        // Config dirty flag
        let cfg_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(if cfg_dirty { WARN_YLW } else { FTR_BG })
            .background_color(FTR_BG).build();
        let _ = Text::new("*CFG*", Point::new(178, 252), cfg_style).draw(d);

        // Version line
        let ver_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10).text_color(LBL_GRAY).background_color(FTR_BG).build();
        let _ = Text::new("Ground Station v3 | ESP32-S3", Point::new(4, 268), ver_style).draw(d);
    }
}
