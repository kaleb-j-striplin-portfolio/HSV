#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use embedded_hal::{delay::DelayNs, digital::OutputPin};
use microbit::{
    adc::{Adc, AdcConfig},
    board::Board,
    hal::{Timer, gpio::Level},
};

/// Number of refresh cycles to stay on a given brightness.
const CYCLES_PER_BRIGHTNESS: usize = 4;
/// Microseconds of duty per brightness step.
const US_PER_BRIGHTNESS: u32 = 4;
/// Number of brightness steps.
const BRIGHTNESSES: u32 = 256;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);
    let pin_r = board.edge.e08.into_push_pull_output(Level::Low);
    let pin_g = board.edge.e09.into_push_pull_output(Level::Low);
    let pin_b = board.edge.e16.into_push_pull_output(Level::Low);
    let mut pin_pot = board.edge.e02.into_push_pull_output(Level::Low);
    let mut adc = Adc::new(board.ADC, AdcConfig::default());
    let mut pins = [pin_r.degrade(), pin_g.degrade(), pin_b.degrade()];
    let mut index = 0;
    let states = [0b100, 0b010, 0b001, 0b011, 0b101, 0b110, 0b111, 0];
    let nstates = states.len();

    loop {
        let state = states[index];
        for brightness in (1..BRIGHTNESSES).chain((1..BRIGHTNESSES).rev()) {
            for _ in 0..CYCLES_PER_BRIGHTNESS {
                for (i, p) in pins.iter_mut().enumerate() {
                    if (state >> (2 - i)) & 1 == 1 {
                        p.set_low().unwrap();
                    } else {
                        p.set_high().unwrap();
                    }
                }
                timer.delay_us(US_PER_BRIGHTNESS * brightness);

                for p in pins.iter_mut() {
                    p.set_high().unwrap();
                }
                timer.delay_us(US_PER_BRIGHTNESS * (BRIGHTNESSES - brightness));
            }
            let adc_reading = adc.read_channel(&mut pin_pot).unwrap();
            rprintln!("adc reading: {}", adc_reading);
        }
        index = (index + 1) % nstates;
    }
}
