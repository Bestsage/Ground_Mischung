use core::cell::RefCell;
use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::Text,
};
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_bus::spi::RefCellDevice;
use esp_hal::{delay::Delay, gpio::Output, spi::master::Spi, spi::FullDuplexMode};
use heapless::String;
use mipidsi::{
    models::ST7789,
    options::{ColorInversion, Orientation},
    Builder,
};

// Import DelayNs trait to enable delay_ms
use embedded_hal::delay::DelayNs;

use crate::menu::{MenuState, Screen};
use crate::RocketState;

/// SharedPin wrapper allows multiple drivers to share the same OutputPin (e.g. DC or RST)
/// by holding a reference to a RefCell.
#[derive(Clone, Copy)]
pub struct SharedPin<'a, P> {
    pub pin: &'a RefCell<P>,
}

impl<'a, P: OutputPin> ErrorType for SharedPin<'a, P> {
    type Error = P::Error;
}

impl<'a, P: OutputPin> OutputPin for SharedPin<'a, P> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.borrow_mut().set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.borrow_mut().set_high()
    }
}

type SpiType<'d> = Spi<'d, esp_hal::peripherals::SPI2, FullDuplexMode>;

// Use RefCellDevice for shared SPI, and SharedPin for DC. RST is manual.
type DisplayType<'d> = mipidsi::Display<
    SPIInterface<RefCellDevice<'d, SpiType<'d>, Output<'d>, Delay>, SharedPin<'d, Output<'d>>>,
    ST7789,
    // Default ResetPin is our local no-op
    NoResetPin,
>;

pub struct NoResetPin;
impl OutputPin for NoResetPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
impl ErrorType for NoResetPin {
    type Error = core::convert::Infallible;
}

// Colors
const BG_COLOR: Rgb565 = Rgb565::new(1, 2, 2);
const HEADER_BG: Rgb565 = Rgb565::new(0, 8, 16);
const OK_GREEN: Rgb565 = Rgb565::new(0, 31, 8);
const WARN_YELLOW: Rgb565 = Rgb565::new(31, 28, 0);
const ERR_RED: Rgb565 = Rgb565::new(31, 4, 4);
const SELECTED_BG: Rgb565 = Rgb565::new(4, 8, 12);

pub struct Driver<'d> {
    display1: DisplayType<'d>,
    display2: DisplayType<'d>,
    frame_count: u32,
    last_screen: Option<Screen>,
}

impl<'d> Driver<'d> {
    pub fn new(
        spi_bus: &'d RefCell<SpiType<'d>>,
        dc_bus: &'d RefCell<Output<'d>>,
        rst_bus: &'d RefCell<Output<'d>>,
        cs1: Output<'d>,
        cs2: Output<'d>,
        delay: &mut Delay,
    ) -> Self {
        // Create SharedPins for DC only
        let dc1 = SharedPin { pin: dc_bus };
        let dc2 = SharedPin { pin: dc_bus };
        // rst is handled manually below

        // Manual Reset for both displays (since they share RST)
        let _ = rst_bus.borrow_mut().set_low();
        delay.delay_ms(10);
        let _ = rst_bus.borrow_mut().set_high();
        delay.delay_ms(150); // Wait for restart

        // Display 1 (CS1)
        let device1 = RefCellDevice::new(spi_bus, cs1, Delay::new());
        let di1 = SPIInterface::new(device1, dc1);
        let mut display1 = Builder::new(ST7789, di1)
            .display_size(240, 280)
            .display_offset(0, 20)
            .orientation(Orientation::default())
            .invert_colors(ColorInversion::Inverted)
            .reset_pin(NoResetPin) // Use our local no-op pin
            .init(delay)
            .unwrap();
        let _ = display1.clear(BG_COLOR);

        // Display 2 (CS2)
        let device2 = RefCellDevice::new(spi_bus, cs2, Delay::new());
        let di2 = SPIInterface::new(device2, dc2);
        let mut display2 = Builder::new(ST7789, di2)
            .display_size(240, 280)
            .display_offset(0, 20)
            .orientation(Orientation::default())
            .invert_colors(ColorInversion::Inverted)
            .reset_pin(NoResetPin) // Use our local no-op pin
            .init(delay)
            .unwrap();
        let _ = display2.clear(BG_COLOR);

        Self {
            display1,
            display2,
            frame_count: 0,
            last_screen: None,
        }
    }

    pub fn update(&mut self, state: &RocketState, menu: &MenuState) {
        self.frame_count = self.frame_count.wrapping_add(1);

        // D1 is static, draw labels only once on frame 1
        let d1_first = self.frame_count == 1;

        // D2 changes w/ menu
        let screen_changed = self.last_screen != Some(menu.current_screen);
        let d2_redraw = screen_changed || d1_first;

        if screen_changed {
            // Only clear D2 logic
            let _ = self.display2.clear(BG_COLOR);
            self.last_screen = Some(menu.current_screen);
        }

        // Initial clear for D1
        if d1_first {
            let _ = self.display1.clear(BG_COLOR);
        }

        // Draw to Display 1 (Main Flight Data)
        // Pass d1_first flag to draw labels only once
        Self::draw_display1_static(&mut self.display1, state, menu, d1_first);

        // Draw to Display 2 (Menu & Config)
        Self::draw_display2_static(&mut self.display2, state, menu, d2_redraw);
    }

    // Display 1: FLIGHT DASHBOARD
    // Shows critical info: RSSI, Battery, Vario, Altitude
    // Display 1: FLIGHT DASHBOARD
    // Shows critical info: RSSI, Battery, Vario, Altitude
    fn draw_display1_static(
        target: &mut DisplayType<'d>,
        state: &RocketState,
        menu: &MenuState,
        draw_labels: bool,
    ) {
        if draw_labels {
            Self::draw_header(target, "FLIGHT DASH", state.link_quality);
        } else {
            // Just update header status icon (link quality)
            Self::draw_header_status(target, state.link_quality);
        }

        // Re-use sensor drawing, passing draw_labels flag
        Self::draw_elrs_sensors(target, state, menu, draw_labels);
    }

    // Helper to update just the status circle without redrawing the whole header
    fn draw_header_status(target: &mut DisplayType<'d>, link_quality: u8) {
        let indicator_color = if link_quality > 80 {
            OK_GREEN
        } else if link_quality > 50 {
            WARN_YELLOW
        } else {
            ERR_RED
        };
        let _ = Circle::new(Point::new(210, 4), 20)
            .into_styled(PrimitiveStyle::with_fill(indicator_color))
            .draw(target);
    }

    // Display 2: MENU & SETTINGS
    // Controlled by encoder.
    // Display 2: MENU & SETTINGS
    // Controlled by encoder.
    fn draw_display2_static(
        target: &mut DisplayType<'d>,
        state: &RocketState,
        menu: &MenuState,
        draw_labels: bool,
    ) {
        if draw_labels {
            // Draw header
            Self::draw_header(target, menu.current_screen.name(), state.link_quality);
        } else {
            Self::draw_header_status(target, state.link_quality);
        }

        // Draw menu content
        match menu.current_screen {
            Screen::ElrsSensors => {
                // On the secondary screen, "ElrsSensors" mode shows detailed extra stats
                Self::draw_detailed_stats(target, state, menu, draw_labels);
            }
            Screen::ElrsConfig => Self::draw_elrs_config(target, menu),
            Screen::Settings => Self::draw_settings(target, menu),
        }

        // Footer with navigation hint (only redraw if needed? actually simple enough to redraw)
        if draw_labels {
            Self::draw_footer(target, menu);
        }
    }

    // Specific for D2 - detailed stats
    fn draw_detailed_stats(
        target: &mut DisplayType<'d>,
        state: &RocketState,
        _menu: &MenuState,
        draw_labels: bool,
    ) {
        let label_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
        // Opaque value style
        let value_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .background_color(BG_COLOR)
            .build();

        if draw_labels {
            let _ = Text::new("GPS Coords:", Point::new(10, 60), label_style).draw(target);
        }

        let mut buf = String::<32>::new();
        let _ = core::fmt::write(&mut buf, format_args!("{:.5}   ", state.lat)); // Pad
        let _ = Text::new(&buf, Point::new(10, 80), value_style).draw(target);

        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:.5}   ", state.lon));
        let _ = Text::new(&buf, Point::new(10, 100), value_style).draw(target);

        // Altitude Max
        if draw_labels {
            let _ = Text::new("Est. Max Alt:", Point::new(10, 140), label_style).draw(target);
        }
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{:.1}m    ", state.alt)); // Placeholder for max logic
        let _ = Text::new(&buf, Point::new(10, 160), value_style).draw(target);
    }

    fn get_theme_accent(theme_idx: u8) -> Rgb565 {
        match theme_idx {
            0 => Rgb565::new(0, 20, 31), // Cyan (Default)
            1 => Rgb565::new(31, 20, 0), // Orange
            2 => Rgb565::new(0, 31, 5),  // Matrix Green
            3 => Rgb565::new(28, 5, 28), // Purple
            _ => Rgb565::new(0, 20, 31),
        }
    }

    fn draw_header(target: &mut DisplayType<'d>, title: &str, link_quality: u8) {
        // Header bar
        let _ = Rectangle::new(Point::new(0, 0), Size::new(240, 28))
            .into_styled(PrimitiveStyle::with_fill(HEADER_BG))
            .draw(target);

        // Title
        let title_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        let _ = Text::new(title, Point::new(8, 20), title_style).draw(target);

        // Link quality indicator
        let indicator_color = if link_quality > 80 {
            OK_GREEN
        } else if link_quality > 50 {
            WARN_YELLOW
        } else {
            ERR_RED
        };
        let _ = Circle::new(Point::new(210, 4), 20)
            .into_styled(PrimitiveStyle::with_fill(indicator_color))
            .draw(target);
    }

    fn draw_footer(target: &mut DisplayType<'d>, menu: &MenuState) {
        let _ = Rectangle::new(Point::new(0, 255), Size::new(240, 25))
            .into_styled(PrimitiveStyle::with_fill(HEADER_BG))
            .draw(target);

        let style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
        let hint = if menu.is_active {
            "HOLD 1.5s -> EXIT"
        } else {
            "HOLD 1.5s -> ENTER"
        };
        let x = (240 - (hint.len() as i32 * 6)) / 2;
        let _ = Text::new(hint, Point::new(x, 270), style).draw(target);
    }

    /// Screen 1: ELRS Sensors - optimized partial redraw
    fn draw_elrs_sensors(
        target: &mut DisplayType<'d>,
        state: &RocketState,
        menu: &MenuState,
        draw_labels: bool,
    ) {
        let mut buf = String::<48>::new();
        // Opaque styles avoid needing clear rects
        let value_style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(Rgb565::WHITE)
            .background_color(BG_COLOR)
            .build();

        let accent = Self::get_theme_accent(menu.theme_idx);
        let header_style = MonoTextStyle::new(&FONT_6X10, accent);

        // === RF LINK ===
        if draw_labels {
            let _ = Text::new("[ RF LINK ]", Point::new(10, 45), header_style).draw(target);
        }

        // UPLINK
        let up_color = if state.uplink_rssi > -60 {
            OK_GREEN
        } else if state.uplink_rssi > -80 {
            WARN_YELLOW
        } else {
            ERR_RED
        };
        let up_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(up_color)
            .background_color(BG_COLOR)
            .build();

        // UP Value
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("UP:{:3}", state.uplink_rssi)); // Pad width
        let _ = Text::new(&buf, Point::new(10, 72), up_style).draw(target);

        // DOWNLINK
        let dn_color = if state.downlink_rssi > -60 {
            OK_GREEN
        } else if state.downlink_rssi > -80 {
            WARN_YELLOW
        } else {
            ERR_RED
        };
        let dn_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(dn_color)
            .background_color(BG_COLOR)
            .build();

        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("DN:{:3}", state.downlink_rssi));
        let _ = Text::new(&buf, Point::new(10, 92), dn_style).draw(target);

        // LQ
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("LQ:{:3}", state.link_quality));
        let lq_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .background_color(BG_COLOR)
            .build();
        let _ = Text::new(&buf, Point::new(130, 72), lq_style).draw(target);

        // === ATTITUDE ===
        if draw_labels {
            let _ = Text::new("[ ATTITUDE ]", Point::new(10, 95), header_style).draw(target);
        }

        let attitude_items = [
            ("P", state.gyro[0]),
            ("R", state.gyro[1]),
            ("Y", state.gyro[2]),
        ];

        let label_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
        // Opaque values for attitude (white on bg)
        // Re-use val_style? Yes.

        for (i, (label, val)) in attitude_items.iter().enumerate() {
            let x = 10 + (i as i32) * 75;
            if draw_labels {
                let _ = Text::new(label, Point::new(x, 115), label_style).draw(target);
            }
            buf.clear();
            let _ = core::fmt::write(&mut buf, format_args!("{:.1}  ", val)); // Pad with spaces
            let _ = Text::new(&buf, Point::new(x + 15, 115), value_style).draw(target);
        }

        // === SENSORS ===
        if draw_labels {
            let _ = Text::new("[ SENSORS ]", Point::new(10, 135), header_style).draw(target);
        }

        let sensor_items = [
            ("Vario", format_args!("{:.1}m/s  ", state.vel), 150),
            ("Batt ", format_args!("{}mV    ", state.batt_voltage), 165),
            ("Sats ", format_args!("{}      ", state.satellites), 180),
        ];

        for (label, val_fmt, y) in sensor_items.iter() {
            if draw_labels {
                let _ = Text::new(label, Point::new(10, *y), label_style).draw(target);
            }
            buf.clear();
            let _ = core::fmt::write(&mut buf, *val_fmt);
            let _ = Text::new(&buf, Point::new(70, *y), value_style).draw(target);
        }

        // === PACKET STATS ===
        if draw_labels {
            let _ = Text::new("[ PACKETS ]", Point::new(10, 205), header_style).draw(target);
        }

        buf.clear();
        let _ = core::fmt::write(
            &mut buf,
            format_args!(
                "RX:{} LS:{} G:{} ",
                state.elrs_packet_count, state.elrs_link_stats_count, state.elrs_gps_count
            ),
        );
        let _ = Text::new(&buf, Point::new(10, 225), value_style).draw(target);
    }

    /// Screen 2: ELRS Config - modifiable settings
    fn draw_elrs_config(target: &mut DisplayType<'d>, menu: &MenuState) {
        let mut buf = String::<32>::new();
        let label_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
        // let value_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE); // Unused
        let accent = Self::get_theme_accent(menu.theme_idx);

        let tlm_ratio_name = match menu.settings.tlm_ratio {
            0 => "Off",
            1 => "1:128",
            2 => "1:64",
            3 => "1:32",
            4 => "1:16",
            5 => "1:8",
            6 => "1:4",
            7 => "1:2",
            _ => "?",
        };

        let rate_name = match menu.settings.packet_rate {
            0 => "25Hz",
            1 => "50Hz",
            2 => "100Hz",
            3 => "150Hz",
            4 => "200Hz",
            5 => "250Hz",
            6 => "500Hz",
            _ => "?",
        };

        let items: [(&str, &str); 3] = [
            ("TLM Ratio", tlm_ratio_name),
            ("Packet Rate", rate_name),
            ("TX Power", ""),
        ];

        for (i, (label, preset)) in items.iter().enumerate() {
            let y = 55 + (i as i32) * 55;
            let is_selected = menu.is_active && menu.selected_item == i as u8;

            // Background
            let bg = if is_selected { SELECTED_BG } else { BG_COLOR };
            let _ = Rectangle::new(Point::new(0, y - 15), Size::new(240, 50))
                .into_styled(PrimitiveStyle::with_fill(bg))
                .draw(target);

            // Label
            let _ = Text::new(label, Point::new(15, y), label_style).draw(target);

            // Value (big)
            buf.clear();
            if i == 2 {
                let _ = core::fmt::write(&mut buf, format_args!("{}%", menu.settings.tx_power));
            } else {
                let _ = core::fmt::write(&mut buf, format_args!("{}", preset));
            }

            let val_color = if is_selected { accent } else { Rgb565::WHITE };
            let val_style = MonoTextStyle::new(&FONT_10X20, val_color);
            let _ = Text::new(&buf, Point::new(15, y + 25), val_style).draw(target);

            // Selection indicator
            if is_selected {
                let _ = Text::new(
                    ">",
                    Point::new(5, y + 25),
                    MonoTextStyle::new(&FONT_10X20, accent),
                )
                .draw(target);
            }
        }

        // Config dirty indicator
        if menu.settings.elrs_config_dirty {
            let dirty_style = MonoTextStyle::new(&FONT_6X10, WARN_YELLOW);
            let _ = Text::new("* SENDING CONFIG *", Point::new(55, 235), dirty_style).draw(target);
        }
    }

    /// Screen 3: Settings - local device settings
    fn draw_settings(target: &mut DisplayType<'d>, menu: &MenuState) {
        let mut buf = String::<32>::new();
        let label_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
        let value_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        let accent = Self::get_theme_accent(menu.theme_idx);

        let theme_name = match menu.theme_idx {
            0 => "Cyan",
            1 => "Orange",
            2 => "Matrix",
            3 => "Purple",
            _ => "?",
        };

        // Theme setting (modifiable)
        let y = 60;
        let is_selected = menu.is_active && menu.selected_item == 0;
        let bg = if is_selected { SELECTED_BG } else { BG_COLOR };

        let _ = Rectangle::new(Point::new(0, y - 15), Size::new(240, 50))
            .into_styled(PrimitiveStyle::with_fill(bg))
            .draw(target);

        let _ = Text::new("Theme", Point::new(15, y), label_style).draw(target);

        let val_color = if is_selected { accent } else { Rgb565::WHITE };
        let val_style = MonoTextStyle::new(&FONT_10X20, val_color);
        let _ = Text::new(theme_name, Point::new(15, y + 25), val_style).draw(target);

        if is_selected {
            let _ = Text::new(
                ">",
                Point::new(5, y + 25),
                MonoTextStyle::new(&FONT_10X20, accent),
            )
            .draw(target);
        }

        // Uptime (read-only)
        let y2 = 130;
        let _ = Rectangle::new(Point::new(0, y2 - 15), Size::new(240, 50))
            .into_styled(PrimitiveStyle::with_fill(BG_COLOR))
            .draw(target);

        let _ = Text::new("Uptime", Point::new(15, y2), label_style).draw(target);
        buf.clear();
        let _ = core::fmt::write(&mut buf, format_args!("{}s", menu.uptime_secs));
        let _ = Text::new(&buf, Point::new(15, y2 + 25), value_style).draw(target);

        // Version info
        let ver_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
        let _ = Text::new("Ground Station v1.1", Point::new(50, 200), ver_style).draw(target);
        let _ = Text::new("ESP32-C3 + ELRS", Point::new(60, 215), ver_style).draw(target);
    }
}
