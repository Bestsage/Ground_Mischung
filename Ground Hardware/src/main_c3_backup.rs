//! Ground Station Firmware for ESP32-C3 Super Mini
//! Menu system with rotary encoder, CRSF to ELRS TX
//! Uses esp-hal 0.21.1

#![no_std]
#![no_main]

extern crate alloc;

use core::cell::RefCell;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io::Write as EmbeddedWrite;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Input, Level, Output, Pull},
    prelude::*,
    spi::master::Spi,
    timer::timg::TimerGroup,
    uart::Uart,
};
use esp_println::println;

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
type UartTxType = esp_hal::uart::UartTx<'static, esp_hal::peripherals::UART1, esp_hal::Async>;
static UART_CRSF: Mutex<RefCell<Option<UartTxType>>> = Mutex::new(RefCell::new(None));

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
    println!("=== Ground Station ESP32-C3 ===");
    println!("[1/6] Peripherals init OK");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    println!("[2/6] Embassy init OK");

    esp_alloc::heap_allocator!(32 * 1024);
    println!("[3/6] Heap allocator OK");

    let io = esp_hal::gpio::Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // SPI2 for display (Shared SPI) - Initialize BEFORE UART to avoid timing conflicts
    let dc = Output::new(io.pins.gpio21, Level::Low);
    let rst = Output::new(io.pins.gpio20, Level::High);
    let cs1 = Output::new(io.pins.gpio10, Level::High);
    let cs2 = Output::new(io.pins.gpio4, Level::High); // Second screen CS

    let spi2 = Spi::new(peripherals.SPI2, 80.MHz(), esp_hal::spi::SpiMode::Mode0)
        .with_sck(io.pins.gpio2)
        .with_mosi(io.pins.gpio3);
    println!("[4/6] SPI2 init OK");

    // Wrap shared resources in RefCell
    let spi_bus = RefCell::new(spi2);
    let dc_bus = RefCell::new(dc);
    let rst_bus = RefCell::new(rst);

    let mut delay = Delay::new();
    let mut display_driver =
        display::Driver::new(&spi_bus, &dc_bus, &rst_bus, cs1, cs2, &mut delay);
    println!("[5/6] Dual Display init OK");

    // UART1 for CRSF to ELRS TX - Initialize AFTER displays
    use esp_hal::uart::config::Config as UartConfig;
    let uart_config = UartConfig::default().baudrate(400_000);
    let uart1 =
        Uart::new_async_with_config(peripherals.UART1, uart_config, io.pins.gpio6, io.pins.gpio7)
            .unwrap();

    let (rx, tx) = uart1.split();
    critical_section::with(|cs| {
        UART_CRSF.borrow_ref_mut(cs).replace(tx);
    });
    println!("[6/6] UART1 CRSF init OK");

    // Rotary encoder pins
    // GPIO1, GPIO5 for rotation signals, GPIO0 for button
    let enc_a = Input::new(io.pins.gpio1, Pull::Up);
    let enc_b = Input::new(io.pins.gpio5, Pull::Up);
    let enc_btn = Input::new(io.pins.gpio0, Pull::Up);

    // NOTE: ARM switch moved to software control via menu
    // let pin_arm = Input::new(io.pins.gpio18, Pull::Up);

    println!("");
    println!("Ground Station Started!");
    println!("Encoder: GPIO1/5, Button: GPIO0");
    println!("CRSF TX: GPIO7");
    println!("");

    // Spawn tasks
    spawner.spawn(crsf_tx_task()).unwrap();
    spawner.spawn(encoder_task(enc_a, enc_b, enc_btn)).unwrap();
    spawner.spawn(crsf_rx_task(rx)).unwrap();

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
            // Send telemetry JSON to PC (50Hz effective rate)
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
            );
            println!("{}", json);
        }

        frame_counter = frame_counter.wrapping_add(1);
        Timer::after(Duration::from_millis(1)).await; // 1kHz loop
    }
}

/// Rx Task
#[embassy_executor::task]
async fn crsf_rx_task(
    mut rx: esp_hal::uart::UartRx<'static, esp_hal::peripherals::UART1, esp_hal::Async>,
) {
    use embassy_futures::select::{select, Either};
    use embedded_io_async::Read;

    // Wait for ELRS module to stabilize before starting RX
    println!("CRSF RX: Waiting 500ms for ELRS module to stabilize...");
    Timer::after(Duration::from_millis(500)).await;

    // Drain any garbage data in UART buffer at startup
    let mut drain_buf = [0u8; 64];
    let mut drained = 0usize;
    loop {
        let drain_result = select(
            rx.read(&mut drain_buf),
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

    println!("CRSF RX task started - GPIO6 RX ready");

    loop {
        // Use select to add a timeout/heartbeat - shorter timeout for responsiveness
        let read_future = rx.read(&mut buf);
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
                        "RX heartbeat #{}: NO DATA on GPIO6 (bytes so far: {})",
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

/// CRSF Transmit task - sends RC channels at 50Hz
#[embassy_executor::task]
async fn crsf_tx_task() {
    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut packet_count: u32 = 0;
    let mut config_send_counter: u8 = 0;

    println!("CRSF TX task started (50Hz)");

    loop {
        ticker.next().await;

        // Check if ELRS config needs to be sent
        let (config_dirty, tlm_ratio, pkt_rate) = critical_section::with(|cs| {
            let menu = MENU_STATE.borrow_ref(cs);
            (
                menu.settings.elrs_config_dirty,
                menu.settings.tlm_ratio,
                menu.settings.packet_rate,
            )
        });

        if config_dirty {
            // Send config packets in sequence SLOWLY (every 50 ticks = 500ms)
            // 0: Device Ping (wake up TX module)
            // 1: Packet Rate
            // 2: TLM Ratio
            // 3: Clear flag

            // Only proceed if packet_count is a multiple of 50
            if packet_count % 50 == 0 {
                config_send_counter = config_send_counter.wrapping_add(1);

                match config_send_counter % 5 {
                    // Use 5 states: 1 (Ping), 2 (Rate), 3 (Tlm), 4 (Done), 0 (Internal Wait)
                    1 => {
                        // First: send Device Ping to wake up TX module
                        println!("ELRS CONFIG: Sending Device Ping to TX module (0xEE)");
                        let ping_frame = crsf::build_device_ping(crsf::CRSF_ADDRESS_TX);
                        critical_section::with(|cs| {
                            if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                                let _ = tx.write_all(&ping_frame);
                            }
                        });
                    }
                    2 => {
                        // Second: send packet rate config
                        println!("ELRS CONFIG: Setting packet rate to index {}", pkt_rate);
                        let config_frame = crsf::build_elrs_packet_rate(pkt_rate);
                        critical_section::with(|cs| {
                            if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                                let _ = tx.write_all(&config_frame);
                            }
                        });
                    }
                    3 => {
                        // Third: send TLM ratio config
                        println!("ELRS CONFIG: Setting TLM ratio to index {}", tlm_ratio);
                        let config_frame = crsf::build_elrs_tlm_ratio(tlm_ratio);
                        critical_section::with(|cs| {
                            if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                                let _ = tx.write_all(&config_frame);
                            }
                        });
                    }
                    4 => {
                        // Done, clear dirty flag
                        critical_section::with(|cs| {
                            MENU_STATE.borrow_ref_mut(cs).settings.elrs_config_dirty = false;
                        });
                        println!("ELRS CONFIG: Config sent, dirty flag cleared");
                        config_send_counter = 0; // Reset
                    }
                    _ => {
                        // Wait state or initial 0
                    }
                }
            }
        } else {
            config_send_counter = 0; // Reset if not dirty
        }

        // Regular RC channels transmission
        let ctrl = critical_section::with(|cs| *CONTROL.borrow_ref(cs));

        // Check scheduling for telemetry packets
        // Full cycle 30 ticks (approx 1.5s at 20Hz loop - Wait, loop is 50Hz!)
        // User said: "Assuming a loop time of ~50ms (20Hz)".
        // BUT code says: `Ticker::every(Duration::from_hz(50));` -> 20ms loop.
        // If the receiving end expects a somewhat slower rate or based on 50ms ticks, I should adjust.
        // If I keep 50Hz loop, "tick" increments every 20ms.
        // User table:
        // GPS ~4Hz. Tick%5==0. 50Hz/5 = 10Hz. Too fast?
        // User assumed 20Hz loop. So 5 ticks = 250ms -> 4Hz.
        // My loop is 50Hz (20ms).
        // To match rates:
        // GPS ~4Hz -> 250ms interval -> Every 12.5 ticks? Round to 12 or 13.
        // Let's change the ticker only for telemetry distribution OR change the modulus.
        // If I change modulus:
        // 50Hz / X = 4Hz => X = 12.5.
        // Let's slow down the ticker for telemetry OR use a separate counter or divider.
        // I will implement a "crsf_tick" that increments at approx 20Hz (every 2.5 real ticks? or just every 50ms).
        // Let's use `telemetry_ticker` separate or just modulo on the 50Hz loop with larger numbers?
        // Simpler: Use a separate counter that updates every 50ms.
        // Or just adjust the scheduler logic to 50Hz base.
        // User wrote: "Assuming a loop time of ~50ms (20Hz)".
        // THIS MEANS THE USER WANTS THE TICKS TO BE 50ms APART.
        // I should probably run the scheduling logic every 50ms (e.g., every 2nd or 3rd loop, or use a separate timer).
        // Let's change `crsf_tx_task` loop to match the user assumption if possible, or handle it properly.
        // However, RC channels (50Hz) are standard. I shouldn't slow down RC to 20Hz.
        // So I will send RC every 20ms (50Hz), but Telemetry frames only on specific ticks of a simulated 20Hz clock.

        // 20ms per loop. We want 50ms ticks. 2.5 loops per tick.
        // Let's do it every 2 loops = 40ms, or every 3 = 60ms?
        // Let's stick to 20Hz target = 50ms.
        // Every 5 loops (100ms) = 2 ticks?
        // Let's run a "telemetry tick" counter.

        packet_count = packet_count.wrapping_add(1);

        // Run telemetry logic every 50ms approx?
        // Let's just use the `packet_count` (50Hz) and scale the modulos?
        // User wants: GPS 4Hz. 50Hz / 12 = ~4.16Hz.
        // User wants: Bat 0.66Hz (every ~1.5s). 50Hz / 75 = 0.66Hz.
        // The pattern provided is `Tick % 30` where Tick is 50ms.
        // So period is 30 * 50ms = 1500ms = 1.5s.
        // I can simulate "Tick" by incrementing a counter every 50ms?
        // Or I can just map the 50Hz counter to the 20Hz requirements.
        // 50Hz count % 75 (1.5s)
        // GPS: ~4Hz.
        // Let's implement a 'telemetry_tick' that increments every 50ms.
        // Since loop is 20ms, we can increment 'telemetry_tick' every 2.5 loops... tricky.
        // Let's just track time.

        // Simpler approach: Send user requested packets at user requested rates using time checks?
        // OR follow the user's specific "Tick % 5" logic assuming I have a 20Hz tick.
        // I'll create a 20Hz sub-event.

        if packet_count % 2 == 0 { // Every 40ms (close enough to 50ms?) No, that's 25Hz.
             // If I do every 3 it's 16.6Hz.
             // Let's use `state_machine_tick` variable.
        }

        // Best approach: Use a separate `Instant` to track "ticks"?
        // Or just map to 50Hz.
        // User Table:
        // Tick % 5 == 0 (GPS) -> every 5 ticks.
        // If I want 4Hz GPS, I need 5 ticks = 250ms => 1 tick = 50ms.
        // So "Tick" is indeed 50ms.

        // Timer for telemetry tick (20Hz / 50ms)
        // Let's execute this block approximately every 50ms.
        // 50Hz loop = 20ms.
        // On loop 0 (0ms): Send Telemetry Tick X
        // On loop 1 (20ms): -
        // On loop 2 (40ms): -
        // On loop 3 (60ms): Send Telemetry Tick X+1 ? (Drift)

        // Let's just add a separate `telemetry_tick` variable and update it based on time.

        // BUT WAIT, I cannot block the RC channel sending (50Hz).
        // So I will send RC channels every loop.
        // I will conditionally interleave a Telemetry packet if it's time.

        // Quick hack: Use a simple decimator.
        // 50Hz loop.
        // 2 loops = 40ms. 3 loops = 60ms. Avg 2.5.
        // Let's float a counter.

        // Or just strictly follow "Tick" as a counter variable, incremented every ~50ms.
        // I'll use a `last_telemetry_time` instant.

        // Below implementation uses a 20Hz (50ms) throttle for the telemetry schedule.

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

            let mut packet_to_send: Option<heapless::Vec<u8, 64>> = None;

            // Scheduler logic from user
            let cycle_30 = tick % 30;
            let cycle_5 = tick % 5;

            if cycle_30 == 29 {
                // Flight Mode
                let mode_str = match ctrl.mode {
                    0 => "MANUAL",
                    1 => "STABILIZE",
                    _ => "AUTO",
                };
                // println!("Sending FlightMode: {}", mode_str);
                packet_to_send = Some(crsf::payload_flight_mode(mode_str));
            } else if cycle_30 == 4 {
                // Barometer
                let (press, temp) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    (rs.press, rs.temp)
                });
                // println!("Sending Baro");
                // rs.press is f32 (hPa usually). Payload expects Pa.
                // rs.temp is f32 (C). Payload expects C.
                packet_to_send = Some(crsf::payload_barometer(press * 100.0, temp));
            } else if cycle_30 == 2 {
                // Battery
                let (volt, batt_rem) = critical_section::with(|cs| {
                    let rs = ROCKET_STATE.borrow_ref(cs);
                    // batt_voltage is u32 mV.
                    (rs.batt_voltage as f32 / 1000.0, 0u8) // TODO: calc remaining
                });
                // println!("Sending Batt");
                packet_to_send = Some(crsf::payload_battery(volt, 0.0, 0.0, batt_rem));
            } else {
                if cycle_5 == 0 {
                    // GPS
                    let (lat, lon, spd, hdg, alt, sats) = critical_section::with(|cs| {
                        let rs = ROCKET_STATE.borrow_ref(cs);
                        (rs.lat, rs.lon, rs.vel, 0.0, rs.alt, rs.satellites)
                    });
                    // println!("Sending GPS");
                    packet_to_send = Some(crsf::payload_gps(lat, lon, spd, hdg, alt, sats));
                } else if cycle_5 == 1 {
                    // Attitude
                    let (p, r, y) = critical_section::with(|cs| {
                        let rs = ROCKET_STATE.borrow_ref(cs);
                        (rs.gyro[0], rs.gyro[1], rs.gyro[2])
                    });
                    // println!("Sending Att");
                    packet_to_send = Some(crsf::payload_attitude(p / 57.29, r / 57.29, y / 57.29));
                // deg to rad
                } else if cycle_5 == 3 {
                    // Vario
                    let (alt, vel) = critical_section::with(|cs| {
                        let rs = ROCKET_STATE.borrow_ref(cs);
                        (rs.alt, rs.vel)
                    });
                    // println!("Sending Vario");
                    packet_to_send = Some(crsf::payload_vario(alt, vel));
                }
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

        // Regular RC channels transmission
        let ctrl = critical_section::with(|cs| *CONTROL.borrow_ref(cs));
        let channels = build_rc_channels(&ctrl);
        let packet = crsf::build_channels_packet(&channels);

        critical_section::with(|cs| {
            if let Some(ref mut tx) = *UART_CRSF.borrow_ref_mut(cs) {
                let _ = tx.write_all(&packet);
            }
        });

        packet_count = packet_count.wrapping_add(1);
        if packet_count % 250 == 0 {
            let arm_status = if ctrl.arm_switch { "ARMED" } else { "DISARMED" };
            println!("CRSF TX (50Hz) - {}", arm_status);
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
