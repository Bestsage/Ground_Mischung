//! Menu system for Ground Station - Simplified 3-screen layout
//! Screen 1: ELRS Sensors (telemetry data)
//! Screen 2: ELRS Config (tlm_ratio, packet_rate, tx_power)
//! Screen 3: Settings (theme, uptime)
//!
//! ESP32-S3: Alt history can be expanded via PSRAM external buffers.
//! The `alt_history` here is a compact graph buffer for display rendering.
//! For full flight logging, use the PSRAM-backed `FlightHistory` in main.rs.

// Graph history buffer size (display pixel width)
pub const GRAPH_WIDTH: usize = 240;

// Extended PSRAM flight history buffer (allocated in PSRAM via alloc::vec)
// At 10Hz sampling, 32768 samples = ~54 minutes of flight data
pub const FLIGHT_HISTORY_SIZE: usize = 32768;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Screen {
    ElrsSensors, // First screen - telemetry data from rocket
    ElrsConfig,  // ELRS configuration (modifiable)
    Settings,    // Local settings (theme)
}

impl Screen {
    pub fn name(&self) -> &'static str {
        match self {
            Screen::ElrsSensors => "ELRS SENSORS",
            Screen::ElrsConfig => "ELRS CONFIG",
            Screen::Settings => "SETTINGS",
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Screen::ElrsSensors => Screen::ElrsConfig,
            Screen::ElrsConfig => Screen::Settings,
            Screen::Settings => Screen::ElrsSensors,
        }
    }

    pub fn prev(&self) -> Self {
        match self {
            Screen::ElrsSensors => Screen::Settings,
            Screen::ElrsConfig => Screen::ElrsSensors,
            Screen::Settings => Screen::ElrsConfig,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ComponentStatus {
    Ok,
    Warning,
    Error,
    NotFound,
    Checking,
}

impl ComponentStatus {
    pub fn symbol(&self) -> &'static str {
        match self {
            ComponentStatus::Ok => "[OK]",
            ComponentStatus::Warning => "[!!]",
            ComponentStatus::Error => "[XX]",
            ComponentStatus::NotFound => "[--]",
            ComponentStatus::Checking => "[..]",
        }
    }
}

#[derive(Clone, Copy)]
pub struct HealthStatus {
    pub display: ComponentStatus,
    pub uart_crsf: ComponentStatus,
    pub elrs_link: ComponentStatus,
    pub gps: ComponentStatus,
    pub imu: ComponentStatus,
    pub baro: ComponentStatus,
    pub flash: ComponentStatus,
    pub battery: ComponentStatus,
}

impl Default for HealthStatus {
    fn default() -> Self {
        Self {
            display: ComponentStatus::Checking,
            uart_crsf: ComponentStatus::Checking,
            elrs_link: ComponentStatus::NotFound,
            gps: ComponentStatus::NotFound,
            imu: ComponentStatus::NotFound,
            baro: ComponentStatus::NotFound,
            flash: ComponentStatus::Checking,
            battery: ComponentStatus::Checking,
        }
    }
}

#[derive(Clone, Copy)]
pub struct ControlSettings {
    pub arm_enabled: bool,
    pub pyro_test_enabled: bool,
    pub abort_armed: bool,
    pub tx_power: u8,    // 0-100%
    pub tlm_ratio: u8,   // ELRS telemetry ratio index: 0=Off, 1=1:128...7=1:2
    pub packet_rate: u8, // ELRS packet rate index: 0=25Hz...6=500Hz
    pub elrs_config_dirty: bool,
}

impl Default for ControlSettings {
    fn default() -> Self {
        Self {
            arm_enabled: false,
            pyro_test_enabled: false,
            abort_armed: false,
            tx_power: 100,
            tlm_ratio: 7,   // 1:2 (good for telemetry)
            packet_rate: 1, // 50Hz
            elrs_config_dirty: false,
        }
    }
}

#[derive(Clone)]
pub struct MenuState {
    pub current_screen: Screen,
    pub is_active: bool,
    pub selected_item: u8,
    pub health: HealthStatus,
    pub settings: ControlSettings,
    pub encoder_position: i32,
    pub uptime_secs: u32,
    pub history_idx: usize,
    pub alt_history: [i16; GRAPH_WIDTH],
    pub theme_idx: u8,
}

impl Default for MenuState {
    fn default() -> Self {
        Self {
            current_screen: Screen::ElrsSensors, // Start on sensors screen
            is_active: false,
            selected_item: 0,
            health: HealthStatus::default(),
            settings: ControlSettings::default(),
            encoder_position: 0,
            uptime_secs: 0,
            history_idx: 0,
            alt_history: [0; GRAPH_WIDTH],
            theme_idx: 0,
        }
    }
}

impl MenuState {
    pub fn push_history(&mut self, alt: f32) {
        self.alt_history[self.history_idx] = alt as i16;
        self.history_idx = (self.history_idx + 1) % GRAPH_WIDTH;
    }

    /// Handle encoder rotation
    pub fn handle_encoder(&mut self, delta: i32) {
        self.encoder_position = self.encoder_position.wrapping_add(delta);

        if !self.is_active {
            // Navigate between screens
            if delta > 0 {
                self.current_screen = self.current_screen.next();
            } else if delta < 0 {
                self.current_screen = self.current_screen.prev();
            }
            self.selected_item = 0; // Reset selection when changing screen
        } else {
            // Navigate inside screen
            let max_items = match self.current_screen {
                Screen::ElrsSensors => 0, // Read-only, no selection
                Screen::ElrsConfig => 2,  // TLM Ratio, Packet Rate, TX Power
                Screen::Settings => 1,    // Theme, (Uptime is read-only)
            };

            if max_items > 0 {
                if delta > 0 {
                    self.selected_item = self.selected_item.saturating_add(1).min(max_items);
                } else if delta < 0 {
                    self.selected_item = self.selected_item.saturating_sub(1);
                }
            }
        }
    }

    /// Short press: modify selected value
    pub fn handle_short_press(&mut self) {
        if !self.is_active {
            return; // Need to enter active mode first
        }

        match self.current_screen {
            Screen::ElrsSensors => {
                // Read-only screen
            }
            Screen::ElrsConfig => {
                match self.selected_item {
                    0 => {
                        // TLM Ratio: cycle 0-7
                        self.settings.tlm_ratio = (self.settings.tlm_ratio + 1) % 8;
                        self.settings.elrs_config_dirty = true;
                    }
                    1 => {
                        // Packet Rate: cycle 0-6
                        self.settings.packet_rate = (self.settings.packet_rate + 1) % 7;
                        self.settings.elrs_config_dirty = true;
                    }
                    2 => {
                        // TX Power: cycle 25, 50, 75, 100
                        self.settings.tx_power = match self.settings.tx_power {
                            25 => 50,
                            50 => 75,
                            75 => 100,
                            _ => 25,
                        };
                    }
                    _ => {}
                }
            }
            Screen::Settings => {
                match self.selected_item {
                    0 => {
                        // Theme: cycle 0-3
                        self.theme_idx = (self.theme_idx + 1) % 4;
                    }
                    _ => {}
                }
            }
        }
    }

    /// Long press: enter/exit active mode
    pub fn handle_long_press(&mut self) {
        self.is_active = !self.is_active;
        self.selected_item = 0;
    }
}
