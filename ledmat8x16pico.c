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

#define N_DISPLAY_MODULES 3
// Seems like 10MHz is around the limit for decently shaped pulses with my
// hardware.
// Delay after TX FIFO empty must be sufficiently long for the higher frequencies
// it seems. Not an elegant solution, but it works for now...
#define TLC59283_TX_FREQ 1000000

#define PIN_LED 25

//                                     left most module    right most module
//              <-- shift direction    left       right    left       right     <-- shift direction
uint16_t test_pattern[2] = {0b1010101010101010, 0b1101000000000111};

/*uint16_t test_frame[N_ROWS][N_DISPLAY_MODULES] = {*/
/*    0xc6c6e6f6decec600,*/
/*};*/

uint16_t frame_buffer[N_ROWS][N_DISPLAY_MODULES + 1];

// Prepare variables to hold references to used PIO, state-machine and PIO
// program offset
PIO pio = pio0;
uint sm_tx = 0;

int dma_chan;

void frame_buffer_insert_hex(uint64_t hex, int n_module, bool r, bool g) {
    for (int row = 0; row < N_ROWS; row++) {
        uint8_t x = 0;
        uint8_t y = 0;
        if (g) {
            x = *((uint8_t *)&hex + row);
        }
        if (r) {
            y = *((uint8_t *)&hex + row);
        }
        uint16_t z;
        // Interleave bits
        // Shamelessly stolen from http://graphics.stanford.edu/~seander/bithacks.html#Interleave64bitOps
        z = ((x * 0x0101010101010101ULL & 0x8040201008040201ULL) * 0x0102040810204081ULL >> 49) &
                0x5555 |
            ((y * 0x0101010101010101ULL & 0x8040201008040201ULL) * 0x0102040810204081ULL >> 48) &
                0xAAAA;

        frame_buffer[row][n_module + 1] = z;
    }
}

void set_row_length(uint16_t length) {
    // Set the row length entries in the frame buffer (all the same value)
    for (int row = 0; row < N_ROWS; row++) {
        // First column actually contains the counter preset values, which are the row length - 1
        frame_buffer[row][0] = length - 1;
    }
}

void init_frame_buffer() {
    // Populate row length headers in frame buffer. They are all the same number, but to ease
    // passing the data via dma we duplicat the header for each row.
    set_row_length(N_DISPLAY_MODULES * 16);
    for (int module = 0; module < N_DISPLAY_MODULES; module++) {
        // for (int row = 0; row < N_ROWS; row++) {
        //     frame_buffer[row][module + 1] = test_pattern[module % 2];
        // }
        frame_buffer_insert_hex(0xc6c6e6f6decec600, 0, 1, 0);
        frame_buffer_insert_hex(0x6666667e66663c00, 1, 1, 1);
        frame_buffer_insert_hex(0x3c66760606663c00, 2, 0, 1);
        // frame_buffer_insert_hex(0x3c66666666663c00, 3, 1, 1);
        // frame_buffer_insert_hex(0x1818183c66666600, 4, 1, 0);
        // frame_buffer_insert_hex(0x6666667e66663c00, 5, 1, 1);
        // frame_buffer_insert_hex(0x0, 6, 0, 1);
    }
}

void rotate_frame_buffer(int n) {
    for (int row = 0; row < N_ROWS; row++) {
        // Left most bits of left most module moved to right side of module
        uint16_t carry = frame_buffer[row][0 + 1] << (16 - n);
        uint16_t next_carry;
        for (int module = N_DISPLAY_MODULES - 1; module >= 0;
             module--) { // Start with right most module
            // Left most bit of current module (to be shifted out) moved to right side of module
            next_carry = frame_buffer[row][module + 1] << (16 - n);
            frame_buffer[row][module + 1] = (frame_buffer[row][module + 1] >> n) | carry;
            carry = next_carry; // Update carry for next module to the shifted out bit
        }
    }
}

bool update_frame_callback(__unused struct repeating_timer *t) {
    rotate_frame_buffer(2); // Rotate two positions (one LED slot as there are two LEDs per slot)
    return true;
}

void row_done_handler() {
    // This callback is called when one row is done (meaning all bits have been shifted out, not
    // just fifo empty!)
    static bool first_run = true;

    static uint8_t current_row = 0;
    static uint8_t prev_row = N_ROWS - 1;

    if (first_run) {
        first_run = false;
    } else {
        gpio_put(PIN_BLANK, 1); // Blank the display
        // Strobe latch pin to latch shifted in value
        gpio_put(PIN_LATCH, 1);
        // For some reason sleep_us here does not work (at least when I have a debugger connected)
        busy_wait_us(10);
        gpio_put(PIN_LATCH, 0);

        // Turn off previous row
        gpio_put(PIN_ROWS_BASE + prev_row, 1);
        // Turn on current row
        gpio_put(PIN_ROWS_BASE + current_row, 0);
        gpio_put(PIN_BLANK, 0); // Un-blank the display

        // Increase row counters
        prev_row = current_row;
        current_row++;

        if (prev_row >= N_ROWS) {
            prev_row = 0;
        }

        if (current_row >= N_ROWS) {
            current_row = 0;
        }
    }
    // Clear the interrupt request
    //dma_hw->ints0 = 1u << dma_chan;

    // Dispatch DMA with the next data
    dma_channel_set_read_addr(dma_chan, frame_buffer[current_row], true);

    // Clear PIO interrupt (and NVIC)
    pio_interrupt_clear(pio, 0);
    irq_clear(PIO0_IRQ_0);
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
        N_DISPLAY_MODULES + 1, // Write the same value many times, then halt and interrupt
        false              // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    // dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    // irq_set_exclusive_handler(DMA_IRQ_0, dma_done);
    // irq_set_enabled(DMA_IRQ_0, true);

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by
    // the hardware
    uint offset_tx = pio_add_program(pio, &tlc59283_tx_program);

    // Have irq trigger an interrupt
    // pis_interrupt0 SUPPOSEDLY! referes to PIO interrupt 0 (the one we use)
    pio_set_irq0_source_enabled(pio, (enum pio_interrupt_source) ((uint) pis_interrupt0 + sm_tx), true);
    // Setup handler to be called on pio interrupt
    // Unique handler should actually be fine?
    irq_add_shared_handler(PIO0_IRQ_0, row_done_handler, 0);
    // Clear interrupt nr zero
    pio_interrupt_clear(pio, 0);
    irq_clear(PIO0_IRQ_0);
    // Enablle PIO IRQ interrupt
    irq_set_enabled(PIO0_IRQ_0, true);

    tlc59283_tx_program_init(pio, sm_tx, offset_tx, PIN_CLK, PIN_DATA, TLC59283_TX_FREQ);

    sleep_ms(10);

    // Setup timer for frame refresh callback
    struct repeating_timer timer;
    add_repeating_timer_ms(50, update_frame_callback, NULL, &timer);

    // Manually call the handler once, to trigger the first transfer
    //dma_handler();

    // Everything else from this point is interrupt-driven. The processor has
    // time to sit and think about its early retirement -- maybe open a bakery?
    while (true) tight_loop_contents();

    // This will free resources and unload our program.
    // Technically the program never reaches this point but keep the line here
    // so I don't forget about this function.
    pio_remove_program_and_unclaim_sm(&tlc59283_tx_program, pio, sm_tx, offset_tx);
}
