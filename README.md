# ASM Playground

Various small examples that program AVR microcontrollers entirely in assembler.

This is the result of using some of my "stay at home" time to (re)experience the
fun of coding directly on hardware.  This project involves both software as well as
hardware, from building the circuits that successfully power-up and run a bare naked
microcontroller chip, to figuring out how to burn code
onto the microcontrollers, and building a suitable set of circuits for each effort.  

The code is entirely in assembler, so interrupt tables, static data, and everything else
are set up explicitly in assembler.

Note that all examples (so far) use ATmega328p chips configured (using an external oscillator)
to run at 16 MHz.

The various examples projects are:

## FirstBurnCode

Very simple code that I use when burning an executable on a new chip to verify basic
operation.  This code:
* Blinks an LED connected to PB0 at 1 Hz.
* Blinking is done using timer-triggered interrupts to toggle PB0 output on and off.

## FirstBurnCodeAlt

An alternate version of the first burn code that makes it easier to confirm (in a crude but not-so-crude way) that the chip is running at about 16 MHz.  This code:
* Blinks two LEDs (PB0 and PB1) at 1 Hz on opposite duty cycles.
* Blinking is done using timer-triggered interrupts to toggle PB0 and PB1 output on and off.
* Both LEDs flash on for 2 seconds at the start to confirm they both work.
* Blinking starts with PB0, alternating every second with PB1.

This code enables a very easy way to check the chip is running at pretty close to 16Mhz timing.
While this approach won't detect a chip running at 16.0000001Mhz, it suffices, if left running
long enough, to easily detect errors in software configuration (e.g., bad fuse bit settings or
wrong scalar) or hardware configuration (bad wiring, wrong oscillator, or wrong type of
oscillator).

The way to use this code to test the chip's clock is:
1. Prepare a stop-watch on your phone.
2. Start the code running, after both LEDs flash off get ready to start the stop-watch.
3. Start the stop-watch when PB0 comes on.  Note that PB0 will be on during even seconds, and PB1 will be on during odd seconds.
4. Leave it running for 24 hrs (or longer)
5. Every so often come back and check that PB0 is still on during even seconds.

## LCDDemo

A more complicated program that runs a plain vanilla 16x2 LCD.  This code:
*  Displays a "Hello World!" greeting message on the top row of the LCD
*  Maintains a simple 16-bit counter that counts elapsed seconds.
*  Displays the number of elapsed seconds in decimal form on the left side
of the bottom row.
*  Displays the number of elapsed seconds in hexadecimal form (with leading
'0x') on the right side of the bottom row.
* As it counts, it posts messages to serve as a 20 second hand-washing timer

## DemoKeypad

A program that interacts with a 4x4 button keypad. This code:
*  Flashes red and green LEDs and the waits for keypad button to be hit.
*  When a key with a non-zero value is hit, the green LED quickly flashes that number of times (e.g., hitting key "3"
causes the green LED to flash 3 times).
*  When the "0" key is hit, there is a single long flash of the red LED.

The keypad values are:

| | Col 1 | Col 2 | Col 3 | Col 4 |
| :---: | :---: | :---: | :---: | :---: |
| **Row 1** | 1 | 2 | 3 | *15* |
| **Row 2** | 4 | 5 | 6 | *14* |
| **Row 3** | 7 | 8 | 9 | *13* |
| **Row 4** | *10* | 0 | *11* | *12* |

Raw key numbers are translated into the above values using a
simple look-up table.

The raw keypad numbers are:

| | Col 1 | Col 2 | Col 3 | Col 4 |
| :---: | :---: | :---: | :---: | :---: |
| **Row 1** | 0 | 1 | 2 | 3 |
| **Row 2** | 4 | 5 | 6 | 7 |
| **Row 3** | 8 | 9 | 10 | 11 |
| **Row 4** | 12 | 13 | 14 | 15 |


## KeypadLcdDemo

A program that connects a 4x4 button keypad to an LCD.  This code:
*  Displays "Key hit:" on the top row of the LCD
*  When a key is hit, displays the value of the key on the bottom row of the LCD.

The keypad values are:

| | Col 1 | Col 2 | Col 3 | Col 4 |
| :---: | :---: | :---: | :---: | :---: |
| **Row 1** | 1 | 2 | 3 | / |
| **Row 2** | 4 | 5 | 6 | x |
| **Row 3** | 7 | 8 | 9 | - |
| **Row 4** | s | 0 | E | + |

The value "s" is shorthand for "+/-" (change sign) and the value "E" is shorthand for
"Enter" (as might appear on an RPN calculator).

Processing of the keypad inputs is done by determining the raw key number and then using a
jump table (implemented as an indirect call) to dispatch to the appropriate function to
process that key (in this case, by display the corresponding value).  This approach is used
instead of a simple look-up table to prepare for more complex processing of key entries
(as might be required for a calculator).

The raw keypad numbers are:

| | Col 1 | Col 2 | Col 3 | Col 4 |
| :---: | :---: | :---: | :---: | :---: |
| **Row 1** | 0 | 1 | 2 | 3 |
| **Row 2** | 4 | 5 | 6 | 7 |
| **Row 3** | 8 | 9 | 10 | 11 |
| **Row 4** | 12 | 13 | 14 | 15 |
