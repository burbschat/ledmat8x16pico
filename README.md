# Raspberry Pi Pico Firmware for Two Color 8x16 LEDs in 8x8 Slots LED Matrix Display Modules
This repository contains the firmware for a Raspberry Pi Pico used to drive the
*LTP-12188M-08* based LED matrix modules for which the hardware design is
available [here](https://github.com/burbschat/ledmat8x16).

This is a work in progress. The code is not yet very clean, but I try my best
to keep it somewhat ordered.
Btw. the code is a `.c` file but I think some syntax I use is techically C++ syntax.

## Features (and Todos)
So far implemented are the following:
* Clocking out serial data using PIO at reasonably high frequencies
    * Runs non-blocking for the duration of a row transmission (data read from frame buffer via DMA)
    * Callback after row is done. Row switch is done in callback.
        * Timers are used to avoid active waits
        * The only remaining active wait is the high duration of the latch
        pulse which is simply one nop (which appears to be the minimum
        required).
* Transmission of frames to the Pico via UART
    * Header data selects frame in frame buffer on the pico
    * Received data written to receive frame buffer non-blocking via DMA
    * Once the buffer is filled, it is latched into the selected location of the frame buffer
    * Plan to implement a state machine for UART communication
        * Have a transmit initialized by some special character
        * Maybe set a timeout for the transmission to make sure we don't get stuck waiting
        * Further states/functionality can be implemented (setting row pulse intervals etc.)
        * A single sided state machine on only the pi where the state can be
        dictated by the PC sending data should be sufficient
* PWM-like adjustment of perceived brightness
    * Also add option for some randomness in the on/off times to avoid annoying
    noises from the LEDs (probably due to thermal expansion)
* A simple example Python script to send a frame via UART

### Notes
* Non-blocking Serial via USB
    * [This thread](https://forums.raspberrypi.com/viewtopic.php?t=354616) contains some useful information
    * Consider TinyUSB for USB CDC
    * `getchar_timeout_us` solution might also be alright, but if possible I'd like to use a interrupt/callback instead of polling
    * For now I'll just use the USB bridge via a second Pico which I use for debugging anyways
