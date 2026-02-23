//! Ground Station Firmware for ESP32-S3 N16R8
//! Dual-core: Core0 = CRSF radio, Core1 = Display + Encoder
//! Dual ST7789 displays via shared SPI bus, PSRAM frame buffers
//! Uses esp-hal 1.0.0-rc.0

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
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
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
    pub lon: f32,
    pub lat: f32,
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
            lon: 2.3522, // Paris
            lat: 48.8566,
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

// Rocket State (Global)
static ROCKET_STATE: Mutex<RefCell<RocketState>> = Mutex::new(RefCell::new(RocketState {
    temp: 0.0,
    press: 0.0,
    alt: 0.0,
    vel: 0.0,
    lon: 0.0,
    lat: 0.0,
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
        tx_power: 100,
        tlm_ratio: 7,   // 1:2
        packet_rate: 6, // 50Hz
        elrs_config_dirty: false,
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
    // SPI2 for dual displays (Shared SPI bus)
    // New ESP32-S3 pinout (avoids GPIO 0, 26-32 for PSRAM Octal):
    //   SCL (Clock)  : GPIO12
    //   SDA (MOSI)   : GPIO11
    //   RES (Reset)   : GPIO14 (shared)
    //   Screen 1 CS  : GPIO10, DC : GPIO9
    //   Screen 2 CS  : GPIO13, DC : GPIO8
    // ================================================================
    let dc1 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let dc2 = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO14, Level::High, OutputConfig::default());
    let cs1 = Output::new(peripherals.GPIO10, Level::High, OutputConfig::default());
    let cs2 = Output::new(peripherals.GPIO13, Level::High, OutputConfig::default());

    // SPI2 with DMA
    let spi_config = SpiConfig::default()
        .with_frequency(Rate::from_mhz(80))
        .with_mode(SpiMode::_0);
    let spi2 = Spi::new(peripherals.SPI2, spi_config)
        .unwrap()
        .with_sck(peripherals.GPIO12)
        .with_mosi(peripherals.GPIO11);
    println!("[4/7] SPI2 init OK");

    // Wrap shared resources in RefCell for display driver
    let spi_bus = RefCell::new(spi2);
    let dc1_cell = RefCell::new(dc1);
    let dc2_cell = RefCell::new(dc2);
    let rst_bus = RefCell::new(rst);

    let mut delay = Delay::new();
    let mut display_driver =
        display::Driver::new(&spi_bus, &dc1_cell, &dc2_cell, &rst_bus, cs1, cs2, &mut delay);
    println!("[5/7] Dual Display init OK");

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
    println!("[6/7] UART1 CRSF init OK (TX:GPIO17 RX:GPIO18)");

    // ================================================================
    // Rotary encoder
    //   Channel A : GPIO4
    //   Channel B : GPIO5
    //   Button    : GPIO6
    // ================================================================
    let enc_a = Input::new(peripherals.GPIO4, InputConfig::default().with_pull(Pull::Up));
    let enc_b = Input::new(peripherals.GPIO5, InputConfig::default().with_pull(Pull::Up));
    let enc_btn = Input::new(peripherals.GPIO6, InputConfig::default().with_pull(Pull::Up));

    println!("[7/7] Encoder init OK (A:GPIO4 B:GPIO5 BTN:GPIO6)");

    println!("");
    println!("Ground Station ESP32-S3 Started!");
    println!("Dual-core: Core0=CRSF, Core1=Display+Encoder");
    println!("Dual ST7789 displays, PSRAM frame buffers");
    println!("");

    // ================================================================
    // DUAL CORE TASK DISTRIBUTION
    // Core 0 (current / PRO_CPU): CRSF TX + RX (radio priority)
    // Core 1 (APP_CPU): Display loop + Encoder
    // ================================================================
    // Spawn CRSF tasks on Core 0 (current core = main)
    spawner.spawn(crsf_tx_task()).unwrap();
    spawner.spawn(crsf_rx_task(rx)).unwrap();
    spawner.spawn(encoder_task(enc_a, enc_b, enc_btn)).unwrap();

    let start_time = Instant::now();
    let mut frame_counter: u32 = 0;

    loop {
        // Update uptime
        let uptime = (start_time.elapsed().as_millis() / 1000) as u32;

        // Sync and Snapshots
        let menu_snapshot = critical_section::with(|cs| {
            let mut menu = MENU_STATE.borrow_ref_mut(cs);
            menu.uptime_secs = uptime;
            // Sync Control
            let mut ctrl = CONTROL.borrow_ref_mut(cs);
            ctrl.arm_switch = menu.settings.arm_enabled;
            ctrl.test_pyro = menu.settings.pyro_test_enabled;
            ctrl.abort = menu.settings.abort_armed;

            // Feed history with REAL altitude from global state
            let alt = ROCKET_STATE.borrow_ref(cs).alt;
            menu.push_history(alt);

            menu.clone()
        });

        // Get Rocket Snapshot
        let state_snapshot = critical_section::with(|cs| {
            let mut s = ROCKET_STATE.borrow_ref_mut(cs);
            // Update arming status from control
            s.armed = CONTROL.borrow_ref(cs).arm_switch;
            // No simulation!
            menu::MenuState::default(); // Dummy call or whatever, just need to return values
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

        display_driver.update(&state_snapshot, &menu_snapshot);

        if frame_counter % 20 == 0 {
            // Send telemetry JSON to PC (~50Hz effective rate)
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
                state_snapshot.lat,
                state_snapshot.lon,
                state_snapshot.temp,
                state_snapshot.press,
            );
            println!("{}", json);
        }

        // Send config status to PC every 500ms (2Hz)
        if frame_counter % 500 == 0 {
            let cfg_json = serial::build_config_json(
                menu_snapshot.settings.tlm_ratio,
                menu_snapshot.settings.packet_rate,
                menu_snapshot.settings.tx_power,
                menu_snapshot.theme_idx,
                menu_snapshot.current_screen as u8,
                menu_snapshot.is_active,
            );
            println!("{}", cfg_json);
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
                                            let lat = i32::from_be_bytes(
                                                payload[0..4].try_into().unwrap_or([0; 4]),
                                            )
                                                as f32
                                                / 10_000_000.0;
                                            let lon = i32::from_be_bytes(
                                                payload[4..8].try_into().unwrap_or([0; 4]),
                                            )
                                                as f32
                                                / 10_000_000.0;
                                            let alt = u16::from_be_bytes(
                                                payload[12..14].try_into().unwrap_or([0; 2]),
                                            )
                                                as f32
                                                - 1000.0;
                                            let sats = payload[14];
                                            println!(
                                                "  GPS: lat={:.5} lon={:.5} alt={:.1}m sats={}",
                                                lat, lon, alt, sats
                                            );

                                            critical_section::with(|cs| {
                                                let mut rs = ROCKET_STATE.borrow_ref_mut(cs);
                                                rs.lat = lat;
                                                rs.lon = lon;
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
                                    crsf::PacketType::ParameterSettingsEntry => {
                                        println!("  ParamEntry received (len={})", payload.len());
                                        // Parse if detailed debugging needed
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

/// High priority encoder task (2kHz polling for fast response)
/// Uses tick accumulation with configurable ticks per detent
#[embassy_executor::task]
async fn encoder_task(enc_a: Input<'static>, enc_b: Input<'static>, enc_btn: Input<'static>) {
    // 500µs polling = 2kHz = much better for catching fast rotations
    let mut ticker = Ticker::every(Duration::from_micros(500));
    let mut btn_press_start: Option<u32> = None;

    // Initial state
    let mut last_enc_state = 0u8;
    if enc_a.is_high() {
        last_enc_state |= 0b01;
    }
    if enc_b.is_high() {
        last_enc_state |= 0b10;
    }

    // Counter in 500µs units
    let mut ticker_count = 0u32;

    // Tick accumulator - adjust TICKS_PER_DETENT based on your encoder
    // Most cheap encoders: 2 or 4 state changes per physical detent
    // Try 2 first, if it skips increase to 4
    let mut tick_accumulator: i32 = 0;
    const TICKS_PER_DETENT: i32 = 2; // Reduced from 4 for more responsive feel

    // Debounce in 500µs units (2 = 1ms debounce)
    let mut last_transition_time = 0u32;
    const DEBOUNCE_TICKS: u32 = 2; // 1ms debounce

    // Look-up table for quadrature decoding
    let enc_lut: [i8; 16] = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0];

    loop {
        ticker.next().await;
        ticker_count = ticker_count.wrapping_add(1);

        // Read encoder state
        let mut curr_state = 0u8;
        if enc_a.is_high() {
            curr_state |= 0b01;
        }
        if enc_b.is_high() {
            curr_state |= 0b10;
        }

        // Process encoder rotation
        if curr_state != last_enc_state {
            let time_since_last = ticker_count.wrapping_sub(last_transition_time);

            if time_since_last >= DEBOUNCE_TICKS {
                last_transition_time = ticker_count;

                let idx = (last_enc_state << 2) | curr_state;
                let val = enc_lut[idx as usize];

                if val != 0 {
                    tick_accumulator += val as i32;

                    // Trigger on full detent
                    if tick_accumulator >= TICKS_PER_DETENT {
                        tick_accumulator = 0; // Reset fully to avoid drift
                        critical_section::with(|cs| {
                            MENU_STATE.borrow_ref_mut(cs).handle_encoder(1);
                        });
                    } else if tick_accumulator <= -TICKS_PER_DETENT {
                        tick_accumulator = 0; // Reset fully
                        critical_section::with(|cs| {
                            MENU_STATE.borrow_ref_mut(cs).handle_encoder(-1);
                        });
                    }
                }
            }
            last_enc_state = curr_state;
        }

        // Button logic (units are 500µs ticks)
        let is_pressed = !enc_btn.is_high();

        if is_pressed && btn_press_start.is_none() {
            btn_press_start = Some(ticker_count);
        } else if !is_pressed && btn_press_start.is_some() {
            let start = btn_press_start.unwrap();
            let duration_ticks = ticker_count.wrapping_sub(start);
            btn_press_start = None;

            // Convert to ms: duration_ticks * 0.5ms
            let duration_ms = duration_ticks / 2;

            if duration_ms > 50 {
                // Debounce 50ms
                if duration_ms > 1500 {
                    // Long Press (> 1.5s)
                    critical_section::with(|cs| {
                        MENU_STATE.borrow_ref_mut(cs).handle_long_press();
                    });
                } else {
                    // Short Press
                    critical_section::with(|cs| {
                        MENU_STATE.borrow_ref_mut(cs).handle_short_press();
                    });
                }
            }
        }
    }
}

/// CRSF Transmit task - sends RC channels at 50Hz + ELRS config
/// Config sequence: Ping → wait 100ms → Write each param with 50ms gaps → verify
#[embassy_executor::task]
async fn crsf_tx_task() {
    let mut ticker = Ticker::every(Duration::from_hz(50)); // True 50Hz for RC
    let mut packet_count: u32 = 0;

    // Config state machine
    // 0 = idle, 1 = ping sent (wait), 2 = rate, 3 = tlm, 4 = power, 5 = done/retry
    let mut config_step: u8 = 0;
    let mut config_step_tick: u32 = 0; // tick when last step was executed
    let mut config_retry_count: u8 = 0;
    const CONFIG_STEP_DELAY: u32 = 5; // 5 ticks = 100ms at 50Hz between config steps
    const CONFIG_PING_DELAY: u32 = 10; // 10 ticks = 200ms after ping
    const MAX_CONFIG_RETRIES: u8 = 3;

    println!("CRSF TX task started (50Hz RC channels) on Core 0");

    loop {
        ticker.next().await;
        packet_count = packet_count.wrapping_add(1);

        // ========== ELRS Config State Machine ==========
        let (config_dirty, tlm_ratio, pkt_rate, tx_power) = critical_section::with(|cs| {
            let menu = MENU_STATE.borrow_ref(cs);
            (
                menu.settings.elrs_config_dirty,
                menu.settings.tlm_ratio,
                menu.settings.packet_rate,
                menu.settings.tx_power,
            )
        });

        if config_dirty && config_step == 0 {
            // Start config sequence
            config_step = 1;
            config_step_tick = packet_count;
            config_retry_count = 0;
            println!("ELRS CONFIG: Starting config sequence (rate={} tlm={} pwr={})",
                     pkt_rate, tlm_ratio, tx_power);
        }

        if config_step > 0 {
            let elapsed = packet_count.wrapping_sub(config_step_tick);

            match config_step {
                1 => {
                    // Step 1: Send Device Ping
                    println!("ELRS CONFIG [1/5]: Device Ping → TX (0xEE)");
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
                    // Step 2: Write Packet Rate (ELRS param 1)
                    println!("ELRS CONFIG [2/5]: Packet Rate → index {}", pkt_rate);
                    let frame = crsf::build_elrs_packet_rate(pkt_rate);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&frame);
                        }
                    });
                    config_step = 3;
                    config_step_tick = packet_count;
                }
                3 if elapsed >= CONFIG_STEP_DELAY => {
                    // Step 3: Write TLM Ratio (ELRS param 2)
                    println!("ELRS CONFIG [3/5]: TLM Ratio → index {}", tlm_ratio);
                    let frame = crsf::build_elrs_tlm_ratio(tlm_ratio);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&frame);
                        }
                    });
                    config_step = 4;
                    config_step_tick = packet_count;
                }
                4 if elapsed >= CONFIG_STEP_DELAY => {
                    // Step 4: Write TX Power (ELRS param 3)
                    // Convert power percentage to ELRS power index:
                    // ELRS power levels: 0=10mW, 1=25mW, 2=50mW, 3=100mW, 4=250mW, 5=500mW, 6=1000mW, 7=2000mW
                    // Map our 25/50/75/100% to reasonable indices
                    let power_idx = match tx_power {
                        0..=25 => 1,   // 25mW
                        26..=50 => 3,  // 100mW
                        51..=75 => 5,  // 500mW
                        _ => 7,        // 2000mW (max)
                    };
                    println!("ELRS CONFIG [4/5]: TX Power → {}% (ELRS index {})", tx_power, power_idx);
                    let frame = crsf::build_elrs_tx_power(power_idx);
                    critical_section::with(|cs| {
                        if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                            let _ = tx.write_all(&frame);
                        }
                    });
                    config_step = 5;
                    config_step_tick = packet_count;
                }
                5 if elapsed >= CONFIG_STEP_DELAY => {
                    // Step 5: Done or retry
                    config_retry_count += 1;
                    if config_retry_count < MAX_CONFIG_RETRIES {
                        // Retry the full sequence for reliability
                        println!("ELRS CONFIG [5/5]: Retry {}/{}", config_retry_count, MAX_CONFIG_RETRIES);
                        config_step = 1;
                        config_step_tick = packet_count;
                    } else {
                        // All retries done, clear dirty flag
                        critical_section::with(|cs| {
                            MENU_STATE.borrow_ref_mut(cs).settings.elrs_config_dirty = false;
                        });
                        println!("ELRS CONFIG: Complete (sent {}x)", MAX_CONFIG_RETRIES);
                        config_step = 0;
                        config_retry_count = 0;
                    }
                }
                _ => {
                    // Waiting for delay to elapse
                }
            }
        }

        // ========== Telemetry Interleaving (20Hz sub-tick) ==========
        // RC at 50Hz, telemetry at 20Hz (every ~50ms = every 2-3 RC ticks)
        static mut LAST_TELEM_TIME: Option<Instant> = None;
        static mut TELEM_TICK: u32 = 0;

        let now = Instant::now();
        let should_tick = unsafe {
            if let Some(last) = LAST_TELEM_TIME {
                if now.duration_since(last) >= Duration::from_millis(50) {
                    LAST_TELEM_TIME = Some(now);
                    true
                } else {
                    false
                }
            } else {
                LAST_TELEM_TIME = Some(now);
                true
            }
        };

        if should_tick {
            let tick = unsafe {
                let t = TELEM_TICK;
                TELEM_TICK = TELEM_TICK.wrapping_add(1);
                t
            };

            let ctrl = critical_section::with(|cs| *CONTROL.borrow_ref(cs));
            let mut packet_to_send: Option<heapless::Vec<u8, 64>> = None;

            let cycle_30 = tick % 30;
            let cycle_5 = tick % 5;

            if cycle_30 == 29 {
                let mode_str = match ctrl.mode {
                    0 => "MANUAL",
                    1 => "STABILIZE",
                    _ => "AUTO",
                };
                packet_to_send = Some(crsf::payload_flight_mode(mode_str));
            } else if cycle_30 == 4 {
                let (press, temp) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    (rs.press, rs.temp)
                });
                packet_to_send = Some(crsf::payload_barometer(press * 100.0, temp));
            } else if cycle_30 == 2 {
                let (volt, batt_rem) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    (rs.batt_voltage as f32 / 1000.0, 0u8)
                });
                packet_to_send = Some(crsf::payload_battery(volt, 0.0, 0.0, batt_rem));
            } else if cycle_5 == 0 {
                let (lat, lon, spd, hdg, alt, sats) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    (rs.lat, rs.lon, rs.vel, 0.0, rs.alt, rs.satellites)
                });
                packet_to_send = Some(crsf::payload_gps(lat, lon, spd, hdg, alt, sats));
            } else if cycle_5 == 1 {
                let (p, r, y) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    (rs.gyro[0], rs.gyro[1], rs.gyro[2])
                });
                packet_to_send = Some(crsf::payload_attitude(p / 57.29, r / 57.29, y / 57.29));
            } else if cycle_5 == 3 {
                let (alt, vel) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    (rs.alt, rs.vel)
                });
                packet_to_send = Some(crsf::payload_vario(alt, vel));
            }

            if let Some(pkt) = packet_to_send {
                let full_frame = crsf::build_telemetry_packet(&pkt);
                critical_section::with(|cs| {
                    if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                        let _ = tx.write_all(&full_frame);
                    }
                });
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
