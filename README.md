# HSV: an HSV to RGB converter for MBv2

Kaleb Striplin 2026

This is a program to run on an MBv2 and uses the expansion board to connect a potentiometer and LED. It uses these components to allow the user to edit the HSV value by adjusting the knob. The HSV value is converted to an RGB value that is set on the LEDs. The RGB values are displayed using PWM driving by a timer and interrupt handler.

### Required hardware:

* MicroBit v2 (MBv2)
* MicroBit GPIO expansion board (edge)
* RGB LED (LED)
* Potentiometer (pot)

### User Interface

Press the A and B buttons on the MBv2 to cycle through editing the H, S, or V value.

Turn the knob on the potentiometer to adjust whichever of the H, S, or V is currently being edited.

### Results and demonstration

As one adjusts the knob for H, the hue (color) will change

As one adjusts the knob for S, the saturation (color purity) will change

As one adjusts the knob for V, the value (brightness) will change

See VIDEO.mov for a demo

## Journal

I believe the commit history is a good record of the steps I took to build this project. However, I will recount a few specific details for clarity.

I began with the hello-rgb repo including an mb2 template and PWM implemented in the main loop.

The first feature to be added was connecting the pot and getting a value from it via the ADC. It was able to find the ADC in the microbit crate here: https://docs.rs/microbit/latest/microbit/adc/type.Adc.html

Knowing I could read the pot, I moved onto the PWM. With the given RgbDisplay structure the bulk of the implementation is the step function called in the interrupt handler. I arrived on a control flow as follows: 

* Handle the tick 0 case for turning on all the LEDs.
* Handle the tick 100 special cases of resetting the tick and the schedule.
* Handle turning off the LEDs that were scheduled to be off on or before the current tick.
* Handle picking the next closest tick in the schedule.
* Handle setting the timer for the appropriate time remaining in the frame.

This implementation was working, but I wasn't seeing the magenta color I was expecting for my test color. I double checked my wiring wasn't backwards. It turned out I had the high vs low backwards. The next commit swapped them and it was working.

The next commit was clamping and normalizing the pot value for setting the HSV, as well as setting up a state tracker for which value was being edited. 

Finally I connected the NBv2 disaply for the UI and set the values appropriately. 

## Build and Run

```rust
cargo embed --release
```

## License

MIT license. See LICENSE.

## Ackowledgements

Thanks to Bart Massey for:

* HSV to RGB conversion code
* RgbDisplay struct and header
* Schematic wiring instructions
* General rust embedded instruction