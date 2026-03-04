//! Ground Station Firmware for ESP32-S3 N16R8
//! Single ST7789 display (CS=GPIO13, DC=GPIO8, RST=GPIO14)
//! Core0: CRSF radio tasks | Main loop: display + serial out

#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;

use core::cell::RefCell;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io::Write as EmbeddedWrite;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    spi::{
        Mode as SpiMode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

mod crsf;
mod display;
mod menu;
mod serial;

use menu::{ComponentStatus, ControlSettings, HealthStatus, MenuState, Screen};

/// Rocket state from telemetry
#[allow(dead_code)]
pub struct RocketState {
    pub temp: f32,
    pub press: f32,
    pub alt: f32,
    pub vel: f32,
    pub lon: i32,  // raw CRSF units: deg * 1e7 (i32 to avoid f32 precision loss)
    pub lat: i32,  // raw CRSF units: deg * 1e7
    pub batt_voltage: u32,
    pub satellites: u8,
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
    pub uplink_rssi: i32,
    pub downlink_rssi: i32,
    pub link_quality: u8,
    pub armed: bool,
    pub flight_mode: u8,
    // ELRS Debug fields
    pub elrs_packet_count: u32,
    pub elrs_byte_count: u32,
    pub elrs_last_radio_id: crsf::RadioId,
    pub elrs_link_stats_count: u32,
    pub elrs_gps_count: u32,
    pub elrs_radio_id_count: u32,
    pub elrs_other_count: u32,
}

impl Default for RocketState {
    fn default() -> Self {
        Self {
            temp: 22.5,
            press: 1013.25,
            alt: 0.0,
            vel: 0.0,
            lon: 23522000, // Paris (2.3522° * 1e7)
            lat: 488566000, // Paris (48.8566° * 1e7)
            batt_voltage: 3850,
            satellites: 0,
            gyro: [0.0; 3],
            accel: [0.0, 0.0, 9.81],
            uplink_rssi: -65,
            downlink_rssi: -65,
            link_quality: 85,
            armed: false,
            flight_mode: 0,
            elrs_packet_count: 0,
            elrs_byte_count: 0,
            elrs_last_radio_id: crsf::RadioId::default(),
            elrs_link_stats_count: 0,
            elrs_gps_count: 0,
            elrs_radio_id_count: 0,
            elrs_other_count: 0,
        }
    }
}

/// Ground station control state
#[derive(Clone, Copy)]
pub struct ControlState {
    pub arm_switch: bool,
    pub mode: u8,
    pub test_pyro: bool,
    pub abort: bool,
}

impl Default for ControlState {
    fn default() -> Self {
        Self {
            arm_switch: false,
            mode: 0,
            test_pyro: false,
            abort: false,
        }
    }
}

// Global control state
static CONTROL: Mutex<RefCell<ControlState>> = Mutex::new(RefCell::new(ControlState {
    arm_switch: false,
    mode: 0,
    test_pyro: false,
    abort: false,
}));

// UART stored in static for task access
// Use RefCell inside Mutex to allow mutability
type UartTxType = esp_hal::uart::UartTx<'static, esp_hal::Blocking>;
static UART_CRSF: Mutex<RefCell<Option<UartTxType>>> = Mutex::new(RefCell::new(None));

// PSRAM Frame Buffers for dual displays (240x280 RGB565 = 134400 bytes each)
pub const FRAME_BUF_SIZE: usize = 240 * 280;
pub type FrameBuffer = Box<[embedded_graphics::pixelcolor::Rgb565; FRAME_BUF_SIZE]>;

// ELRS Discovered Configuration (populated at boot via CRSF parameter discovery)
static ELRS_CONFIG: Mutex<RefCell<crsf::ElrsConfig>> = Mutex::new(RefCell::new(crsf::ElrsConfig {
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
}));

// Rocket State (Global)
static ROCKET_STATE: Mutex<RefCell<RocketState>> = Mutex::new(RefCell::new(RocketState {
    temp: 0.0,
    press: 0.0,
    alt: 0.0,
    vel: 0.0,
    lon: 0,
    lat: 0,
    batt_voltage: 0,
    satellites: 0,
    gyro: [0.0; 3],
    accel: [0.0; 3],
    uplink_rssi: 0,
    downlink_rssi: 0,
    link_quality: 0,
    armed: false,
    flight_mode: 0,
    elrs_packet_count: 0,
    elrs_byte_count: 0,
    elrs_last_radio_id: crsf::RadioId {
        sub_type: 0,
        timing_offset_us: 0,
        interval_us: 0,
        phase: 0,
    },
    elrs_link_stats_count: 0,
    elrs_gps_count: 0,
    elrs_radio_id_count: 0,
    elrs_other_count: 0,
}));

// Global menu state
static MENU_STATE: Mutex<RefCell<MenuState>> = Mutex::new(RefCell::new(MenuState {
    current_screen: Screen::ElrsSensors, // Start on sensors screen
    selected_item: 0,
    health: HealthStatus {
        display: ComponentStatus::Ok,
        uart_crsf: ComponentStatus::Ok,
        elrs_link: ComponentStatus::NotFound,
        gps: ComponentStatus::NotFound,
        imu: ComponentStatus::NotFound,
        baro: ComponentStatus::NotFound,
        flash: ComponentStatus::Ok,
        battery: ComponentStatus::Ok,
    },
    settings: ControlSettings {
        arm_enabled: false,
        pyro_test_enabled: false,
        abort_armed: false,
        tx_power_idx: 3,       // mid power (~100mW on typical modules)
        tx_power_max_idx: 0,
        tlm_ratio: 7,          // 1:2
        packet_rate: 6,        // 500Hz
        elrs_config_dirty: true, // auto-push defaults at boot
    },
    is_active: false,
    encoder_position: 0,
    uptime_secs: 0,
    history_idx: 0,
    alt_history: [0; menu::GRAPH_WIDTH],
    theme_idx: 0,
}));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    println!("=== Ground Station ESP32-S3 N16R8 ===");
    println!("[1/7] Peripherals init OK");

    // Heap allocator: 72KB SRAM (ESP32-S3 has much more internal RAM than C3)
    esp_alloc::heap_allocator!(size: 72 * 1024);
    println!("[2/7] Heap allocator OK");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    println!("[3/7] Embassy init OK");

    // ================================================================
    // SPI2 for single display
    //   SCL (Clock)  : GPIO12
    //   SDA (MOSI)   : GPIO11
    //   RES (Reset)   : GPIO14
    //   Screen CS    : GPIO13
    //   Screen DC    : GPIO8
    // ================================================================
    let dc  = Output::new(peripherals.GPIO8,  Level::Low,  OutputConfig::default());
    let rst = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    let cs  = Output::new(peripherals.GPIO13, Level::High, OutputConfig::default());

    // SPI2
    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(80))
        .with_mode(SpiMode::_0);
    let spi2 = Spi::new(peripherals.SPI2, spi_config)
        .unwrap()
        .with_sck(peripherals.GPIO12)
        .with_mosi(peripherals.GPIO11);
    println!("[4/5] SPI2 init OK");

    let spi_bus = RefCell::new(spi2);
    let dc_cell  = RefCell::new(dc);
    let rst_bus  = RefCell::new(rst);

    let mut delay = Delay::new();
    let mut display_driver =
        display::Driver::new(&spi_bus, &dc_cell, &rst_bus, cs, &mut delay);
    println!("[5/5] Display init OK (CS:GPIO13 DC:GPIO8 RST:GPIO14)");

    // ================================================================
    // UART1 for CRSF to ELRS TX
    //   TX : GPIO17
    //   RX : GPIO18
    // ================================================================
    let uart_config = UartConfig::default().with_baudrate(420000);
    let uart1 = Uart::new(peripherals.UART1, uart_config)
        .unwrap()
        .with_rx(peripherals.GPIO18)
        .with_tx(peripherals.GPIO17)
        .into_async();

    let (rx, tx) = uart1.split();
    // Store TX as Blocking (Async is !Send, can't live in static Mutex)
    let tx_blocking = tx.into_blocking();
    critical_section::with(|cs| {
        UART_CRSF.borrow_ref_mut(cs).replace(tx_blocking);
    });
    // UART1 init OK (TX:GPIO17 RX:GPIO18)
    println!("[3/5] UART1 CRSF init OK (TX:GPIO17 RX:GPIO18)");

    println!("");
    println!("Ground Station ESP32-S3 Started!");
    println!("Single screen, CRSF radio, serial telemetry");
    println!("");

    // ================================================================
    // DUAL CORE TASK DISTRIBUTION
    // Core 0 (current / PRO_CPU): CRSF TX + RX (radio priority)
    // Core 1 (APP_CPU): Display loop + Encoder
    // ================================================================
    spawner.spawn(crsf_tx_task()).unwrap();
    spawner.spawn(crsf_rx_task(rx)).unwrap();

    let start_time = Instant::now();
    let mut frame_counter: u32 = 0;

    loop {
        // Update uptime
        let uptime = (start_time.elapsed().as_millis() / 1000) as u32;

        // Keep uptime in menu state (used elsewhere) and sync arm control
        critical_section::with(|cs| {
            let mut menu = MENU_STATE.borrow_ref_mut(cs);
            menu.uptime_secs = uptime;
        });

        // Get Rocket Snapshot
        let state_snapshot = critical_section::with(|cs| {
            let s = ROCKET_STATE.borrow_ref(cs);
            RocketState {
                temp: s.temp,
                press: s.press,
                alt: s.alt,
                vel: s.vel,
                lon: s.lon,
                lat: s.lat,
                batt_voltage: s.batt_voltage,
                satellites: s.satellites,
                gyro: s.gyro,
                accel: s.accel,
                uplink_rssi: s.uplink_rssi,
                downlink_rssi: s.downlink_rssi,
                link_quality: s.link_quality,
                armed: s.armed,
                flight_mode: s.flight_mode,
                elrs_packet_count: s.elrs_packet_count,
                elrs_byte_count: s.elrs_byte_count,
                elrs_last_radio_id: s.elrs_last_radio_id,
                elrs_link_stats_count: s.elrs_link_stats_count,
                elrs_gps_count: s.elrs_gps_count,
                elrs_radio_id_count: s.elrs_radio_id_count,
                elrs_other_count: s.elrs_other_count,
            }
        });

        let cfg_dirty = critical_section::with(|cs| {
            MENU_STATE.borrow_ref(cs).settings.elrs_config_dirty
        });

        display_driver.update(&state_snapshot, uptime, state_snapshot.armed, cfg_dirty);

        if frame_counter % 2 == 0 {
            // Send telemetry JSON to PC (~500Hz effective rate)
            let json = serial::build_telemetry_json(
                state_snapshot.uplink_rssi,
                state_snapshot.downlink_rssi,
                state_snapshot.link_quality,
                state_snapshot.gyro[0],
                state_snapshot.gyro[1],
                state_snapshot.gyro[2],
                state_snapshot.vel,
                state_snapshot.batt_voltage,
                state_snapshot.satellites,
                state_snapshot.alt,
                state_snapshot.elrs_packet_count,
                state_snapshot.lat, // i32 raw (deg * 1e7)
                state_snapshot.lon, // i32 raw (deg * 1e7)
                state_snapshot.temp,
                state_snapshot.press,
            );
            println!("{}", json);
        }

        // Send config status to PC every 500ms (2Hz)
        if frame_counter % 500 == 0 {
            let cfg_json = serial::build_config_json(
                critical_section::with(|cs| MENU_STATE.borrow_ref(cs).settings.tlm_ratio),
                critical_section::with(|cs| MENU_STATE.borrow_ref(cs).settings.packet_rate),
                critical_section::with(|cs| MENU_STATE.borrow_ref(cs).settings.tx_power_idx),
                0u8,  // theme unused
                0u8,  // screen unused
                false,
            );
            println!("{}", cfg_json);
        }

        // Send ELRS discovered params to PC every 2s (once discovery is done)
        if frame_counter % 2000 == 500 {
            let elrs_snap = critical_section::with(|cs| *ELRS_CONFIG.borrow_ref(cs));
            if elrs_snap.discovery_done && elrs_snap.power_count > 0 {
                let pwr_idx = critical_section::with(|cs| {
                    MENU_STATE.borrow_ref(cs).settings.tx_power_idx
                });
                let elrs_json = serial::build_elrs_info_json(
                    &elrs_snap.power_levels_mw,
                    elrs_snap.power_count,
                    pwr_idx,
                    elrs_snap.discovery_done,
                );
                println!("{}", elrs_json);
            }
        }

        frame_counter = frame_counter.wrapping_add(1);
        Timer::after(Duration::from_millis(1)).await; // 1kHz loop
    }
}

/// Rx Task
#[embassy_executor::task]
async fn crsf_rx_task(
    mut rx: esp_hal::uart::UartRx<'static, esp_hal::Async>,
) {
    use embassy_futures::select::{select, Either};

    // Wait for ELRS module to stabilize before starting RX
    println!("CRSF RX: Waiting 500ms for ELRS module to stabilize...");
    Timer::after(Duration::from_millis(500)).await;

    // Drain any garbage data in UART buffer at startup
    let mut drain_buf = [0u8; 64];
    let mut drained = 0usize;
    loop {
        let drain_result = select(
            rx.read_async(&mut drain_buf),
            Timer::after(Duration::from_millis(50)),
        )
        .await;
        match drain_result {
            Either::First(Ok(n)) => {
                drained += n;
            }
            _ => break, // Timeout or error - buffer is clear
        }
    }
    if drained > 0 {
        println!("CRSF RX: Drained {} garbage bytes from UART", drained);
    }

    let mut parser = crsf::CrsfParser::new();
    // Larger buffer for more efficient reading at 420kbaud
    let mut buf = [0u8; 64];
    let mut byte_count: u32 = 0;
    let mut packet_count: u32 = 0;
    let mut heartbeat: u32 = 0;
    let mut bytes_since_last_packet: u32 = 0;

    println!("CRSF RX task started - GPIO18 RX ready");

    loop {
        // Use select to add a timeout/heartbeat - shorter timeout for responsiveness
        let read_future = rx.read_async(&mut buf);
        let timeout_future = Timer::after(Duration::from_millis(500));

        match select(read_future, timeout_future).await {
            Either::First(result) => {
                match result {
                    Ok(n) if n > 0 => {
                        // Process all received bytes
                        for i in 0..n {
                            byte_count = byte_count.wrapping_add(1);
                            bytes_since_last_packet = bytes_since_last_packet.wrapping_add(1);

                            // Print FIRST 20 bytes to confirm data is arriving
                            if byte_count <= 20 {
                                println!("RX byte #{}: 0x{:02X}", byte_count, buf[i]);
                            } else if byte_count % 1000 == 0 {
                                println!("RX total: {} bytes, {} pkts", byte_count, packet_count);
                            }

                            // Reset parser if stuck (too many bytes without valid packet)
                            // CRSF max frame is 64 bytes, so 200+ bytes without packet = corrupted state
                            if bytes_since_last_packet > 200 {
                                parser = crsf::CrsfParser::new();
                                bytes_since_last_packet = 0;
                                // Don't spam logs
                                if byte_count % 500 == 0 {
                                    println!("CRSF parser reset - sync lost");
                                }
                            }

                            if let Some((ptype, payload)) = parser.push_byte(buf[i]) {
                                bytes_since_last_packet = 0;
                                packet_count = packet_count.wrapping_add(1);
                                println!(
                                    "CRSF PKT #{}: type={:?} len={}",
                                    packet_count,
                                    ptype,
                                    payload.len()
                                );

                                // Handle Packet
                                match ptype {
                                    crsf::PacketType::LinkStatistics => {
                                        if payload.len() >= 10 {
                                            let up_rssi = payload[0] as i8;
                                            let dn_rssi = payload[7] as i8;
                                            let up_lq = payload[2];
                                            println!(
                                                "  LinkStats: UP={}dBm DN={}dBm LQ={}%",
                                                up_rssi, dn_rssi, up_lq
                                            );

                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.uplink_rssi = up_rssi as i32;
                                                rs.downlink_rssi = dn_rssi as i32;
                                                rs.link_quality = up_lq;
                                                rs.elrs_link_stats_count =
                                                    rs.elrs_link_stats_count.wrapping_add(1);
                                                rs.elrs_packet_count = packet_count;
                                                rs.elrs_byte_count = byte_count;

                                                let mut ms = MENU_STATE.borrow_ref_mut(cs);
                                                ms.health.elrs_link = if up_lq > 0 {
                                                    ComponentStatus::Ok
                                                } else {
                                                    ComponentStatus::Error
                                                };
                                            });
                                        }
                                    }
                                    crsf::PacketType::Gps => {
                                        if payload.len() >= 15 {
                                            // Store raw i32 (deg * 1e7) to avoid f32 precision loss
                                            let lat_raw = i32::from_be_bytes(
                                                payload[0..4].try_into().unwrap_or([0; 4]),
                                            );
                                            let lon_raw = i32::from_be_bytes(
                                                payload[4..8].try_into().unwrap_or([0; 4]),
                                            );
                                            let alt = u16::from_be_bytes(
                                                payload[12..14].try_into().unwrap_or([0; 2]),
                                            )
                                                as f32
                                                - 1000.0;
                                            let sats = payload[14];
                                            println!(
                                                "  GPS: lat={} lon={} (raw*1e7) alt={:.1}m sats={}",
                                                lat_raw, lon_raw, alt, sats
                                            );

                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.lat = lat_raw;
                                                rs.lon = lon_raw;
                                                rs.alt = alt;
                                                rs.satellites = sats;
                                                rs.elrs_gps_count =
                                                    rs.elrs_gps_count.wrapping_add(1);

                                                let mut ms = MENU_STATE.borrow_ref_mut(cs);
                                                ms.health.gps = if sats > 3 {
                                                    ComponentStatus::Ok
                                                } else {
                                                    ComponentStatus::Warning
                                                };
                                            });
                                        }
                                    }
                                    crsf::PacketType::Vario => {
                                        // Payload: 2 bytes alt + 2 bytes vspeed? Or custom?
                                        // User said: Altitude (dm), Vertical Speed (cm/s)
                                        // 2 bytes alt_dm (u16/i16?), 2 bytes vspeed (i16) -> 4 bytes total
                                        if payload.len() >= 4 {
                                            let _alt_dm = u16::from_be_bytes(
                                                payload[0..2].try_into().unwrap_or([0; 2]),
                                            );
                                            let vspd_cms = i16::from_be_bytes(
                                                payload[2..4].try_into().unwrap_or([0; 2]),
                                            );
                                            let vspd = vspd_cms as f32 / 100.0;

                                            // println!("  Vario: vspd={:.2}m/s", vspd); // Reduce spam
                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.vel = vspd;
                                                rs.elrs_other_count =
                                                    rs.elrs_other_count.wrapping_add(1);
                                            });
                                        }
                                    }
                                    crsf::PacketType::Barometer => {
                                        // Payload: Pressure (Pa) u32 + Temp (0.01C) i16
                                        if payload.len() >= 6 {
                                            let press_pa = u32::from_be_bytes(
                                                payload[0..4].try_into().unwrap_or([0; 4]),
                                            );
                                            let temp_c = i16::from_be_bytes(
                                                payload[4..6].try_into().unwrap_or([0; 2]),
                                            )
                                                as f32
                                                / 100.0;

                                            // println!("  Baro: {}Pa {:.2}C", press_pa, temp_c);
                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.press = press_pa as f32 / 100.0; // Store as hPa mostly? Or keep Pa.
                                                                                    // RocketState uses press: f32. Let's assume hPa usually, but if value is large it's Pa.
                                                                                    // The default 1013.25 implies hPa.
                                                rs.press = press_pa as f32 / 100.0;
                                                rs.temp = temp_c;
                                                rs.elrs_other_count =
                                                    rs.elrs_other_count.wrapping_add(1);
                                            });
                                        }
                                    }
                                    crsf::PacketType::RadioId => {
                                        if let Some(radio_id) =
                                            crsf::RadioId::from_payload(&payload)
                                        {
                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.elrs_last_radio_id = radio_id;
                                                rs.elrs_radio_id_count =
                                                    rs.elrs_radio_id_count.wrapping_add(1);
                                            });
                                            // Only log occasionally to avoid spam (every 100th packet)
                                            if packet_count % 100 == 0 {
                                                println!(
                                                "  RadioId: sub=0x{:02X} timing={}us interval={}us phase={}",
                                                radio_id.sub_type,
                                                radio_id.timing_offset_us,
                                                radio_id.interval_us,
                                                radio_id.phase
                                            );
                                            }
                                        }
                                    }
                                    crsf::PacketType::BatterySensor => {
                                        // Payload: voltage(2) + current(2) + fuel(3) + remaining(1) = 8 bytes
                                        if payload.len() >= 8 {
                                            let voltage = u16::from_be_bytes(
                                                payload[0..2].try_into().unwrap_or([0; 2]),
                                            )
                                                as u32
                                                * 100; // Convert to mV
                                            let _current = u16::from_be_bytes(
                                                payload[2..4].try_into().unwrap_or([0; 2]),
                                            );
                                            let remaining = payload[7];
                                            println!("  Battery: {}mV rem={}%", voltage, remaining);

                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.batt_voltage = voltage;
                                                rs.elrs_other_count =
                                                    rs.elrs_other_count.wrapping_add(1);
                                            });
                                        }
                                    }
                                    crsf::PacketType::Attitude => {
                                        // Payload: pitch(2) + roll(2) + yaw(2) = 6 bytes (rad * 10000)
                                        if payload.len() >= 6 {
                                            let pitch = i16::from_be_bytes(
                                                payload[0..2].try_into().unwrap_or([0; 2]),
                                            )
                                                as f32
                                                / 10000.0;
                                            let roll = i16::from_be_bytes(
                                                payload[2..4].try_into().unwrap_or([0; 2]),
                                            )
                                                as f32
                                                / 10000.0;
                                            let yaw = i16::from_be_bytes(
                                                payload[4..6].try_into().unwrap_or([0; 2]),
                                            )
                                                as f32
                                                / 10000.0;

                                            // Convert rad to degrees for display
                                            let pitch_deg = pitch * 57.2958;
                                            let roll_deg = roll * 57.2958;
                                            let yaw_deg = yaw * 57.2958;
                                            println!(
                                                "  Attitude: P={:.1} R={:.1} Y={:.1}",
                                                pitch_deg, roll_deg, yaw_deg
                                            );

                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.gyro = [pitch_deg, roll_deg, yaw_deg];
                                                rs.elrs_other_count =
                                                    rs.elrs_other_count.wrapping_add(1);
                                            });
                                        }
                                    }
                                    crsf::PacketType::FlightMode => {
                                        // Payload: null-terminated string
                                        if !payload.is_empty() {
                                            // Find null terminator or use all bytes
                                            let end = payload
                                                .iter()
                                                .position(|&b| b == 0)
                                                .unwrap_or(payload.len());
                                            if let Ok(mode_str) =
                                                core::str::from_utf8(&payload[..end])
                                            {
                                                println!("  FlightMode: {}", mode_str);
                                            }
                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.elrs_other_count =
                                                    rs.elrs_other_count.wrapping_add(1);
                                            });
                                        }
                                    }

                                    crsf::PacketType::ParameterWrite => {
                                        if payload.len() >= 4 {
                                            let dest = payload[0];
                                            let origin = payload[1];
                                            let param_num = payload[2];
                                            let val = payload[3];
                                            println!(
                                            "  ParamWrite: Dest=0x{:02X} Orig=0x{:02X} Param={} Val={}",
                                            dest, origin, param_num, val
                                        );
                                        }
                                    }
                                    crsf::PacketType::DeviceInfo => {
                                        // Parse DeviceInfo to get field count
                                        if let Some(field_count) = crsf::parse_device_info(&payload) {
                                            println!("  DeviceInfo: {} params available", field_count);
                                            critical_section::with(|cs| {
                                                let mut elrs = ELRS_CONFIG.borrow_ref_mut(cs);
                                                elrs.device_found = true;
                                                elrs.total_params = field_count;
                                            });
                                        } else {
                                            println!("  DeviceInfo: parse failed (len={})", payload.len());
                                            // Still mark as found even if parse fails
                                            critical_section::with(|cs| {
                                                let mut elrs = ELRS_CONFIG.borrow_ref_mut(cs);
                                                elrs.device_found = true;
                                                if elrs.total_params == 0 {
                                                    elrs.total_params = 15; // fallback
                                                }
                                            });
                                        }
                                    }
                                    crsf::PacketType::ParameterSettingsEntry => {
                                        // Parse parameter entry for discovery
                                        if let Some(param) = crsf::parse_parameter_entry(&payload) {
                                            println!(
                                                "  ParamEntry #{}: type={} parent={} opts={} val={} power={} rate={} tlm={} mw={}",
                                                param.param_num, param.data_type, param.parent,
                                                param.option_count, param.value,
                                                param.is_power, param.is_rate, param.is_tlm, param.has_mw
                                            );

                                            critical_section::with(|cs| {
                                                let mut elrs = ELRS_CONFIG.borrow_ref_mut(cs);

                                                // Identify power parameter (TEXT_SELECTION with mW in options)
                                                if param.is_power && param.has_mw && param.power_count > 0 {
                                                    elrs.power_param_num = param.param_num;
                                                    elrs.power_count = param.power_count;
                                                    elrs.power_current_idx = param.value;
                                                    elrs.power_levels_mw = param.power_levels;
                                                    println!(
                                                        "  → POWER discovered: param={} count={} current={} levels={:?}",
                                                        param.param_num, param.power_count,
                                                        param.value, &param.power_levels[..param.power_count as usize]
                                                    );

                                                    // Sync to menu settings
                                                    let mut ms = MENU_STATE.borrow_ref_mut(cs);
                                                    ms.settings.tx_power_max_idx = param.power_count.saturating_sub(1);
                                                    ms.settings.tx_power_idx = param.value;
                                                }

                                                // Identify rate parameter
                                                if param.is_rate && param.data_type == crsf::CRSF_PARAM_TYPE_TEXT_SELECTION {
                                                    elrs.rate_param_num = param.param_num;
                                                    elrs.rate_count = param.option_count;
                                                    elrs.rate_current_idx = param.value;
                                                    println!(
                                                        "  → RATE discovered: param={} count={} current={}",
                                                        param.param_num, param.option_count, param.value
                                                    );
                                                }

                                                // Identify TLM ratio parameter
                                                if param.is_tlm && param.data_type == crsf::CRSF_PARAM_TYPE_TEXT_SELECTION {
                                                    elrs.tlm_param_num = param.param_num;
                                                    elrs.tlm_count = param.option_count;
                                                    elrs.tlm_current_idx = param.value;
                                                    println!(
                                                        "  → TLM discovered: param={} count={} current={}",
                                                        param.param_num, param.option_count, param.value
                                                    );
                                                }
                                            });
                                        } else {
                                            println!("  ParamEntry received (len={}) - parse failed", payload.len());
                                        }
                                    }
                                    _ => {
                                        println!(
                                            "  (unhandled packet type: 0x{:02X})",
                                            ptype as u8
                                        );
                                        critical_section::with(|cs| {
                                            let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                            rs.elrs_other_count =
                                                rs.elrs_other_count.wrapping_add(1);
                                        });
                                    }
                                }
                            }
                        }
                    }
                    Ok(_) => {
                        // 0 bytes read - shouldn't happen but handle gracefully
                    }
                    Err(e) => {
                        println!("CRSF RX UART error: {:?}", e);
                        // On error, reset parser state
                        parser = crsf::CrsfParser::new();
                        bytes_since_last_packet = 0;
                    }
                }
            }
            Either::Second(_) => {
                // Timeout - no data received in 500ms
                heartbeat = heartbeat.wrapping_add(1);
                if heartbeat <= 5 {
                    println!(
                        "RX heartbeat #{}: NO DATA on GPIO18 (bytes so far: {})",
                        heartbeat, byte_count
                    );
                }
                // Reset parser on timeout in case it's stuck
                parser = crsf::CrsfParser::new();
                bytes_since_last_packet = 0;
            }
        }
    }
}


/// CRSF Transmit task - sends RC channels at 50Hz + ELRS config + boot discovery
/// Boot: Ping → read all params → discover power/rate/tlm
/// Config: Ping → write each param with delays → retry
#[embassy_executor::task]
async fn crsf_tx_task() {
    let mut ticker = Ticker::every(Duration::from_hz(50)); // True 50Hz for RC
    let mut packet_count: u32 = 0;

    // ===== Discovery state machine =====
    // 0=wait boot, 1=ping sent, 2=wait device info, 3=reading params, 4=done
    let mut discovery_state: u8 = 0;
    let mut discovery_tick: u32 = 0;
    let mut discovery_param_idx: u8 = 1;
    const DISCOVERY_BOOT_WAIT: u32 = 100;  // 100 ticks = 2s at 50Hz
    const DISCOVERY_PING_WAIT: u32 = 25;   // 500ms after ping
    const DISCOVERY_READ_WAIT: u32 = 10;   // 200ms between param reads

    // ===== Config state machine =====
    let mut config_step: u8 = 0;
    let mut config_step_tick: u32 = 0;
    let mut config_retry_count: u8 = 0;
    const CONFIG_STEP_DELAY: u32 = 8; // 160ms - give module more time to process
    const CONFIG_PING_DELAY: u32 = 15; // 300ms - wait longer for ping response
    const MAX_CONFIG_RETRIES: u8 = 5; // more retries for reliability

    println!("CRSF TX task started (50Hz RC + ELRS discovery) on Core 0");

    loop {
        ticker.next().await;
        packet_count = packet_count.wrapping_add(1);

        // ========== Boot Discovery Sequence ==========
        if discovery_state < 4 {
            match discovery_state {
                0 => {
                    // Wait for boot
                    if packet_count >= DISCOVERY_BOOT_WAIT {
                        println!("ELRS DISCOVERY: Sending Device Ping...");
                        let ping = crsf::build_device_ping(crsf::CRSF_ADDRESS_TX);
                        critical_section::with(|cs| {
                            if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                                let _ = tx.write_all(&ping);
                            }
                        });
                        discovery_state = 1;
                        discovery_tick = packet_count;
                    }
                }
                1 => {
                    // Wait for DeviceInfo response
                    let elapsed = packet_count.wrapping_sub(discovery_tick);
                    if elapsed >= DISCOVERY_PING_WAIT {
                        let (found, total) = critical_section::with(|cs| {
                            let elrs = ELRS_CONFIG.borrow_ref(cs);
                            (elrs.device_found, elrs.total_params)
                        });
                        if found && total > 0 {
                            println!("ELRS DISCOVERY: Device found, {} params. Starting read...", total);
                            discovery_state = 3;
                            discovery_param_idx = 1;
                            discovery_tick = packet_count;
                        } else {
                            // Retry ping
                            println!("ELRS DISCOVERY: No response, retrying ping...");
                            let ping = crsf::build_device_ping(crsf::CRSF_ADDRESS_TX);
                            critical_section::with(|cs| {
                                if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                                    let _ = tx.write_all(&ping);
                                }
                            });
                            discovery_tick = packet_count;

                            // After 3 retries, assume 15 params and proceed
                            if elapsed >= DISCOVERY_PING_WAIT * 3 {
                                println!("ELRS DISCOVERY: No DeviceInfo, assuming 15 params");
                                critical_section::with(|cs| {
                                    let mut elrs = ELRS_CONFIG.borrow_ref_mut(cs);
                                    if elrs.total_params == 0 {
                                        elrs.total_params = 15;
                                    }
                                });
                                discovery_state = 3;
                                discovery_param_idx = 1;
                                discovery_tick = packet_count;
                            }
                        }
                    }
                }
                3 => {
                    // Read parameters one by one
                    let elapsed = packet_count.wrapping_sub(discovery_tick);
                    let total = critical_section::with(|cs| {
                        ELRS_CONFIG.borrow_ref(cs).total_params
                    });

                    if elapsed >= DISCOVERY_READ_WAIT {
                        if discovery_param_idx <= total && discovery_param_idx <= 20 {
                            // Send ParameterRead request
                            let frame = crsf::build_parameter_read(discovery_param_idx, 0);
                            critical_section::with(|cs| {
                                if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                                    let _ = tx.write_all(&frame);
                                }
                            });
                            println!("ELRS DISCOVERY: Reading param {}/{}", discovery_param_idx, total);
                            discovery_param_idx += 1;
                            discovery_tick = packet_count;
                        } else {
                            // All params read
                            critical_section::with(|cs| {
                                ELRS_CONFIG.borrow_ref_mut(cs).discovery_done = true;
                            });
                            let elrs_snap = critical_section::with(|cs| {
                                *ELRS_CONFIG.borrow_ref(cs)
                            });
                            println!("ELRS DISCOVERY: Complete!");
                            println!("  Power: param={} count={} levels={:?} current={}",
                                elrs_snap.power_param_num, elrs_snap.power_count,
                                &elrs_snap.power_levels_mw[..elrs_snap.power_count as usize],
                                elrs_snap.power_current_idx);
                            println!("  Rate: param={} count={} current={}",
                                elrs_snap.rate_param_num, elrs_snap.rate_count, elrs_snap.rate_current_idx);
                            println!("  TLM: param={} count={} current={}",
                                elrs_snap.tlm_param_num, elrs_snap.tlm_count, elrs_snap.tlm_current_idx);
                            discovery_state = 4; // done

                            // Auto re-trigger config write with discovered param numbers
                            // This ensures correct params are used even if config already ran with fallbacks
                            critical_section::with(|cs| {
                                MENU_STATE.borrow_ref_mut(cs).settings.elrs_config_dirty = true;
                            });
                            config_step = 0;
                            config_retry_count = 0;
                            println!("ELRS CONFIG: Re-triggering with discovered params");
                        }
                    }
                }
                _ => {} // shouldn't reach here
            }
        }

        // ========== ELRS Config Write State Machine ==========
        let (config_dirty, tlm_ratio, pkt_rate, tx_power_idx) = critical_section::with(|cs| {
            let menu = MENU_STATE.borrow_ref(cs);
            (
                menu.settings.elrs_config_dirty,
                menu.settings.tlm_ratio,
                menu.settings.packet_rate,
                menu.settings.tx_power_idx,
            )
        });

        // Get discovered param numbers (use fallback defaults if not discovered)
        // ELRS 3.x typical param order: 1=Rate, 2=TLM, 3=SwitchMode, 4=AntMode, 5=TXPower(folder), 6=MaxPower, 7=Dynamic
        let (power_pnum, rate_pnum, tlm_pnum) = critical_section::with(|cs| {
            let elrs = ELRS_CONFIG.borrow_ref(cs);
            (
                if elrs.power_param_num > 0 { elrs.power_param_num } else { 6 },
                if elrs.rate_param_num > 0 { elrs.rate_param_num } else { 1 },
                if elrs.tlm_param_num > 0 { elrs.tlm_param_num } else { 2 },
            )
        });

        // Start config after 3s boot delay (150 ticks at 50Hz), don't require discovery
        if config_dirty && config_step == 0 && packet_count >= 150 {
            config_step = 1;
            config_step_tick = packet_count;
            config_retry_count = 0;
            println!("ELRS CONFIG: Starting (rate[p{}]={} tlm[p{}]={} pwr[p{}]={})",
                     rate_pnum, pkt_rate, tlm_pnum, tlm_ratio, power_pnum, tx_power_idx);
        }

        if config_step > 0 {
            let elapsed = packet_count.wrapping_sub(config_step_tick);

            match config_step {
                1 => {
                    let ping = crsf::build_device_ping(crsf::CRSF_ADDRESS_TX);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&ping);
                        }
                    });
                    config_step = 2;
                    config_step_tick = packet_count;
                }
                2 if elapsed >= CONFIG_PING_DELAY => {
                    // Write Packet Rate using discovered param number
                    let frame = crsf::build_parameter_write(rate_pnum, pkt_rate);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&frame);
                        }
                    });
                    println!("ELRS CONFIG: Rate param {} → {}", rate_pnum, pkt_rate);
                    config_step = 3;
                    config_step_tick = packet_count;
                }
                3 if elapsed >= CONFIG_STEP_DELAY => {
                    // Write TLM Ratio using discovered param number
                    let frame = crsf::build_parameter_write(tlm_pnum, tlm_ratio);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&frame);
                        }
                    });
                    println!("ELRS CONFIG: TLM param {} → {}", tlm_pnum, tlm_ratio);
                    config_step = 4;
                    config_step_tick = packet_count;
                }
                4 if elapsed >= CONFIG_STEP_DELAY => {
                    // Write TX Power using discovered param number - DIRECT INDEX
                    // tx_power_idx is already the ELRS index into the discovered power levels
                    let frame = crsf::build_parameter_write(power_pnum, tx_power_idx);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&frame);
                        }
                    });
                    // Look up mW value for logging
                    let mw = critical_section::with(|cs| {
                        let elrs = ELRS_CONFIG.borrow_ref(cs);
                        if (tx_power_idx as usize) < elrs.power_count as usize {
                            elrs.power_levels_mw[tx_power_idx as usize]
                        } else {
                            0
                        }
                    });
                    println!("ELRS CONFIG: Power param {} → idx {} ({}mW)", power_pnum, tx_power_idx, mw);
                    config_step = 5;
                    config_step_tick = packet_count;
                }
                5 if elapsed >= CONFIG_STEP_DELAY => {
                    config_retry_count += 1;
                    if config_retry_count < MAX_CONFIG_RETRIES {
                        println!("ELRS CONFIG: Retry {}/{}", config_retry_count, MAX_CONFIG_RETRIES);
                        config_step = 1;
                        config_step_tick = packet_count;
                    } else {
                        critical_section::with(|cs| {
                            MENU_STATE.borrow_ref_mut(cs).settings.elrs_config_dirty = false;
                        });
                        println!("ELRS CONFIG: Complete ({}x)", MAX_CONFIG_RETRIES);
                        config_step = 0;
                        config_retry_count = 0;
                    }
                }
                _ => {} // waiting for delay
            }
        }

        // ========== RC Channels (every tick = 50Hz) ==========
        let ctrl = critical_section::with(|cs| *CONTROL.borrow_ref(cs));
        let channels = build_rc_channels(&ctrl);
        let packet = crsf::build_channels_packet(&channels);

        critical_section::with(|cs| {
            if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                let _ = tx.write_all(&packet);
            }
        });

        if packet_count % 250 == 0 {
            let arm_status = if ctrl.arm_switch { "ARMED" } else { "DISARMED" };
            println!("CRSF TX (50Hz) - {} - pkts:{}", arm_status, packet_count);
        }
    }
}

/// Convert control state to 16 RC channels
fn build_rc_channels(ctrl: &ControlState) -> [u16; 16] {
    let mut channels = [992u16; 16];

    // Ch5: ARM
    channels[4] = if ctrl.arm_switch { 1811 } else { 172 };

    // Ch6: Mode
    channels[5] = match ctrl.mode {
        0 => 172,
        1 => 992,
        _ => 1811,
    };

    // Ch7: Pyro test
    channels[6] = if ctrl.test_pyro { 1811 } else { 172 };

    // Ch8: Abort
    channels[7] = if ctrl.abort { 1811 } else { 172 };

    channels
}
