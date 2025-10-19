# Raspberry Pi Pico Firmware for Two Color 8x16 LEDs in 8x8 Slots LED Matrix Display Modules
This repository contains the firmware for a Raspberry Pi Pico used to drive the
*LTP-12188M-08* based LED matrix modules for which the hardware design is
available [here](https://github.com/burbschat/ledmat8x16).

This is a work in progress. The code is not yet very clean, but I try my best
to keep it somewhat ordered.
Btw. the code is a `.c` file but I think some syntax I use is techically C++ syntax. Forgive me.

## Features
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
* Transmission of frames to the Pico via USB serial (TinyUSB CDC device)
    * Uses the same protocol as the UART
    * Much faster than the UART!
    * Cannot be run entirely interrupt drive, *but* we have another core in the
    pico (or the MCU on the pico to be precise). The TinyUSB loop is thus run
    on the second core and does not interfere with the main loop.
* PWM-like adjustment of perceived brightness
    * Also added option for some randomness in the on/off times to avoid annoying
    noises from the LEDs (probably due to thermal expansion)
* A simple example Python script to send a frame via serial for a single row display
* A not so simple Python script to send a frame for a multi-row snaked display
    * Takes care of arranging data in the correct order. This is done on the
    host PC as this is much easier and allows the pico to just read buffers in
    the order they are clocked out to the shift registers, which is the simpler
    solution.

## Todos (that I've come to think of so far)
* Implement a state machine for serial communication?
    * Have a transmit initialized by some special character
    * Maybe set a timeout for the transmission to make sure we don't get stuck waiting
    * Further states/functionality can be implemented (setting row pulse intervals etc.)
    * A single sided state machine on only the pi where the state can be
    dictated by the PC sending data should be sufficient
