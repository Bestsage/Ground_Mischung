//! Firmware ULTRA minimal - sans Embassy, juste une boucle
//! ESP32-S3 N16R8

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

use esp_backtrace as _;
use esp_println::println;
use esp_hal::timer::timg::TimerGroup;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    println!("=== ESP32-S3 N16R8 Boot OK! ===");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut counter = 0u32;

    loop {
        println!("Heartbeat #{}", counter);
        counter = counter.wrapping_add(1);
        Timer::after(Duration::from_millis(1000)).await;
    }
}
