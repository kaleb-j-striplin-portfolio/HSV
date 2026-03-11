#![no_main]
#![no_std]

use cortex_m_rt::entry;
use critical_section_lock_mut::LockMut;
use panic_rtt_target as _;
use rtt_target::rtt_init_print;

use embedded_hal::digital::{InputPin, OutputPin};
use microbit::{
    adc::{Adc, AdcConfig},
    board::Board,
    display::blocking::Display,
    hal::{
        Timer,
        gpio::{self, Level},
        pac::{self, interrupt},
    },
};

use hsv::Hsv;

const POT_CLAMP_MIN: i16 = 10;

const POT_CLAMP_MAX: i16 = 16370;

const STEPS_PER_FRAME: f32 = 100f32;

const US_PER_STEP: u32 = 100;

enum Editor {
    H,
    S,
    V,
}

fn normalize_adc_reading(adc_reading: i16) -> f32 {
    match adc_reading {
        adc_reading if adc_reading <= POT_CLAMP_MIN => 0.0f32,
        adc_reading if adc_reading >= POT_CLAMP_MAX => 1.0f32,
        _ => (adc_reading as f32) / (POT_CLAMP_MAX as f32),
    }
}

struct RgbDisplay {
    // What tick of the frame are we currently on?
    // Setting to 0 starts a new frame.
    tick: u32,
    // What ticks should R, G, B LEDs turn off at?
    schedule: [u32; 3],
    // Schedule to start at next frame.
    next_schedule: Option<[u32; 3]>,
    // R, G, and B pins.
    rgb_pins: [gpio::Pin<gpio::Output<gpio::PushPull>>; 3],
    // Timer used to reach next tick.
    timer0: Timer<pac::TIMER0>,
}

impl RgbDisplay {
    fn new(
        rgb_pins: [gpio::Pin<gpio::Output<gpio::PushPull>>; 3],
        timer0: Timer<pac::TIMER0>,
    ) -> Self {
        Self {
            tick: 0,
            schedule: [0, 0, 0],
            next_schedule: None,
            rgb_pins,
            timer0,
        }
    }

    /// Set up a new schedule, to be started next frame.
    fn set(&mut self, hsv: &Hsv) {
        let rgb = hsv.to_rgb();
        let r_steps = (rgb.r * STEPS_PER_FRAME) as u32;
        let g_steps = (rgb.g * STEPS_PER_FRAME) as u32;
        let b_steps = (rgb.b * STEPS_PER_FRAME) as u32;
        self.next_schedule = Some([r_steps, g_steps, b_steps]);
    }

    /// Take the next frame update step. Called at startup
    /// and then from the timer interrupt handler.
    fn step(&mut self) {
        self.timer0.enable_interrupt();

        // special case for tick 100
        // reset the tick and pick the next schedule
        if self.tick == (STEPS_PER_FRAME as u32) {
            self.tick = 0;
            self.schedule = match self.next_schedule {
                Some(schedule) => schedule,
                None => self.schedule,
            };
        }

        // special case for tick 0
        // start frame with all LEDs on
        if self.tick == 0 {
            for p in &mut self.rgb_pins {
                p.set_low().unwrap();
            }
        }

        let mut next_interrupt_tick = 100;
        for t in self.schedule.into_iter().enumerate() {
            // turn off the rgbs below the current tick according to the schedule
            if self.tick >= t.1 {
                self.rgb_pins[t.0].set_high().unwrap();
            }
            // find the next timer value to set
            else if t.1 < next_interrupt_tick {
                next_interrupt_tick = t.1;
            }
        }

        // set the timer update the tick
        let next_interrupt_time = (next_interrupt_tick - self.tick) * US_PER_STEP;
        self.timer0.reset_event();
        self.timer0.start(next_interrupt_time);
        self.tick = next_interrupt_tick;
    }
}

/// The RgbDisplay. Accessible from both the interrupt handler
/// and the main program.
static RGB_DISPLAY: LockMut<RgbDisplay> = LockMut::new();

#[interrupt]
fn TIMER0() {
    RGB_DISPLAY.with_lock(|display| display.step());
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();
    // timer0 for the interrupts
    let timer0 = Timer::new(board.TIMER0);
    // timer1 for the blocking display
    let mut timer1 = Timer::new(board.TIMER1);

    // buttons and display pins for mb2 display UI
    let mut button_a = board.buttons.button_a;
    let mut button_b = board.buttons.button_b;
    let mut mb2_display: Display = Display::new(board.display_pins);

    // RGB pins from the edge connection
    let pin_r = board.edge.e08.into_push_pull_output(Level::Low);
    let pin_g = board.edge.e09.into_push_pull_output(Level::Low);
    let pin_b = board.edge.e16.into_push_pull_output(Level::Low);
    let pins = [pin_r.degrade(), pin_g.degrade(), pin_b.degrade()];

    // potentiometer pin and the ADC for reading it.
    let mut pin_pot = board.edge.e02.into_push_pull_output(Level::Low);
    let mut adc = Adc::new(board.ADC, AdcConfig::default());

    // mb2 display values for the UI
    let leds_h: [[u8; 5]; 5] = [
        [1, 0, 0, 0, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 0, 0, 0, 1],
    ];

    let leds_s: [[u8; 5]; 5] = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1],
        [0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1],
    ];

    let leds_v: [[u8; 5]; 5] = [
        [1, 0, 0, 0, 1],
        [1, 0, 0, 0, 1],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 1, 0, 0],
    ];

    let rgb_display = RgbDisplay::new(pins, timer0);
    RGB_DISPLAY.init(rgb_display);

    RGB_DISPLAY.with_lock(|display| display.step());

    // Set up the NVIC to handle interrupts.
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER0) };
    pac::NVIC::unpend(pac::Interrupt::TIMER0);

    // This tracks the state of the editor in the loop
    // The editor itself keeps track of:
    // which component of Hsv is being edited
    // the mb2 matching display to inform the user
    // and the hsv values
    let mut editor_state: (Editor, [[u8; 5]; 5], Hsv) = (
        Editor::H,
        leds_h,
        Hsv {
            h: 0.9,
            s: 0.75,
            v: 0.8,
        },
    );

    loop {
        // Check for button press and change the edited component of HSV if so.
        let pressed_a: bool = button_a.is_low().unwrap();
        if pressed_a {
            editor_state = match editor_state {
                (Editor::H, _, hsv) => (Editor::V, leds_v, hsv),
                (Editor::S, _, hsv) => (Editor::H, leds_h, hsv),
                (Editor::V, _, hsv) => (Editor::S, leds_s, hsv),
            };
        }

        let pressed_b: bool = button_b.is_low().unwrap();
        if pressed_b {
            editor_state = match editor_state {
                (Editor::H, _, hsv) => (Editor::S, leds_s, hsv),
                (Editor::S, _, hsv) => (Editor::V, leds_v, hsv),
                (Editor::V, _, hsv) => (Editor::H, leds_h, hsv),
            };
        }

        // Read the potentiometer value and use it to set the value of the edited component of HSV.
        let adc_reading = adc.read_channel(&mut pin_pot).unwrap();
        let normalized_adc_reading = normalize_adc_reading(adc_reading);

        // Convert the current floating HSV to an integer RGB schedule to be displayed next frame.
        match editor_state.0 {
            Editor::H => editor_state.2.h = normalized_adc_reading,
            Editor::S => editor_state.2.s = normalized_adc_reading,
            Editor::V => editor_state.2.v = normalized_adc_reading,
        }

        RGB_DISPLAY.with_lock(|display| display.set(&editor_state.2));

        // Block in the MB2 LED display showing the correct component letter for 100 ms.
        mb2_display.show(&mut timer1, editor_state.1, 100);
    }
}
