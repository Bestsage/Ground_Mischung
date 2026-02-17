//! Firmware ULTRA minimal - sans Embassy, juste une boucle

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::delay::Delay;
use esp_println::println;

#[esp_hal::entry]
fn main() -> ! {
    let _ = esp_hal::init(esp_hal::Config::default());
    println!("=== ESP32-C3 Boot OK! ===");

    let delay = Delay::new();
    let mut counter = 0u32;

    loop {
        println!("Heartbeat #{}", counter);
        counter = counter.wrapping_add(1);
        delay.delay_millis(1000);
    }
}
