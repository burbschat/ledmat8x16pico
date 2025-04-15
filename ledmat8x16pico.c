#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "tlc59283.pio.h"
#include <stdio.h>
#include <string.h>

// Both pins must be on the same GPIO group (0->31 or 16->47) as "No single PIO
// instance can interact with both pins 0->15 or 32->47 at the same time." for
// the rp2350.
#define PIN_CLK 2
#define PIN_DATA 3
#define PIN_LATCH 4
#define PIN_BLANK 5
#define PIN_ROWS_BASE 6
#define N_ROWS 8

#define N_DISPLAY_MODULES 2
// Seems like 10MHz is around the limit for decently shaped pulses with my
// hardware.
// Delay after TX FIFO empty must be sufficiently long for the higher frequencies
// it seems. Not an elegant solution, but it works for now...
#define TLC59283_TX_FREQ 1000000

#define PIN_LED 25

//                                     left most module    right most module
//              <-- shift direction    left       right    left       right     <-- shift direction
uint16_t test_pattern[2] = {0b1010101010101010, 0b1101000000000111};

uint16_t frame_buffer[N_ROWS][N_DISPLAY_MODULES];

// Prepare variables to hold references to used PIO, state-machine and PIO
// program offset
PIO pio = pio0;
uint sm_tx = 0;

int dma_chan;

void init_frame_buffer() {
    for (int module = 0; module < N_DISPLAY_MODULES; module++) {
        for (int row = 0; row < N_ROWS; row++) {
            frame_buffer[row][module] = test_pattern[module % 2];
        }
    }
}

void rotate_frame_buffer(int n) {
    for (int row = 0; row < N_ROWS; row++) {
        // Left most bits of left most module moved to right side of module
        uint16_t carry = frame_buffer[row][0] >> (16 - n);
        uint16_t next_carry;
        for (int module = N_DISPLAY_MODULES - 1; module >= 0; module--) { // Start with right most module
            // Left most bit of current module (to be shifted out) moved to right side of module
            next_carry = frame_buffer[row][module] >> (16 - n);
            frame_buffer[row][module] = (frame_buffer[row][module] << n) | carry;
            carry = next_carry; // Update carry for next module to the shifted out bit
        }
    }
}

bool update_frame_callback(__unused struct repeating_timer *t) {
    rotate_frame_buffer(2); // Rotate two positions (one LED slot as there are two LEDs per slot)
    return true;
}

void dma_handler() {
    static bool first_run = true;

    static uint8_t current_row = 0;
    static uint8_t next_row = 1;

    if (first_run) {
        first_run = false;
    } else {
        // TODO: Try to somehow call this callback on TX FIFO empty (which should imply that the dma
        // is finished). This however does appear to be non-trivial as, while there are registers
        // to check if we are stalled on empty TX FIFO (e.g. TXSTALL), they cannot directly call an
        // interrupt. The best solution I can think of is to have a separate PIO sm that polls the
        // register and calls sets an IRQ flag when it signals that the TX sm is stalled.
        while (!pio_sm_is_tx_fifo_empty(pio, sm_tx)) {
        }
        busy_wait_us(
            5); // Apparently we must wait a little here for the data to properly propagate?
        gpio_put(PIN_BLANK, 1); // Blank the display
        // Strobe latch pin to latch shifted in value
        gpio_put(PIN_LATCH, 1);
        // For some reason sleep_us here does not work (at least when I have a debugger connected)
        busy_wait_us(10);
        gpio_put(PIN_LATCH, 0);

        // Turn off current row
        gpio_put(PIN_ROWS_BASE + current_row, 1);
        // Turn on next row
        gpio_put(PIN_ROWS_BASE + next_row, 0);
        gpio_put(PIN_BLANK, 0); // Un-blank the display

        // Increase row counters
        next_row++;
        current_row++;

        if (next_row >= N_ROWS) {
            next_row = 0;
        }

        if (current_row >= N_ROWS) {
            current_row = 0;
        }
    }
    // Clear the interrupt request
    dma_hw->ints0 = 1u << dma_chan;
    // Dispatch DMA with the next data
    dma_channel_set_read_addr(dma_chan, frame_buffer[current_row], true);
}

int main() {
    stdio_init_all();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    gpio_init(PIN_LATCH);
    gpio_set_dir(PIN_LATCH, GPIO_OUT);

    gpio_init(PIN_BLANK);
    gpio_set_dir(PIN_BLANK, GPIO_OUT);
    // Must be pulled low. Leaving it floating did cause problems (blank always asserted).
    gpio_put(PIN_BLANK, 0);

    for (int i = 0; i < N_ROWS; i++) {
        gpio_init(PIN_ROWS_BASE + i);
        gpio_set_dir(PIN_ROWS_BASE + i, GPIO_OUT);
        gpio_put(PIN_ROWS_BASE + i, 1); // Initialize with all rows off (pins high)
    }

    // Initialize frame buffer
    init_frame_buffer();

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by
    // the hardware
    uint offset_tx = pio_add_program(pio, &tlc59283_tx_program);

    tlc59283_tx_program_init(pio, sm_tx, offset_tx, PIN_CLK, PIN_DATA, TLC59283_TX_FREQ);

    sleep_ms(10);

    // Setup DMA
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16); // One module has 16 columns
    channel_config_set_read_increment(&c, true); // Increment read address after each written block
    // Use data request signal from PIO0 TX FIFO (must be matched to used pio!)
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);

    dma_channel_configure(
        dma_chan, &c,
        &pio0_hw->txf[0],  // Write address (only need to set this once)
        NULL,              // Don't provide a read address yet
        N_DISPLAY_MODULES, // Write the same value many times, then halt and interrupt
        false              // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Setup timer for frame refresh callback
    struct repeating_timer timer;
    add_repeating_timer_ms(50, update_frame_callback, NULL, &timer);

    // Manually call the handler once, to trigger the first transfer
    dma_handler();

    // Everything else from this point is interrupt-driven. The processor has
    // time to sit and think about its early retirement -- maybe open a bakery?
    while (true) tight_loop_contents();

    // This will free resources and unload our program.
    // Technically the program never reaches this point but keep the line here
    // so I don't forget about this function.
    pio_remove_program_and_unclaim_sm(&tlc59283_tx_program, pio, sm_tx, offset_tx);
}
