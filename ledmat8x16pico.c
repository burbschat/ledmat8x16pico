#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "tlc59283.pio.h"

// Both pins must be on the same GPIO group (0->31 or 16->47) as "No single PIO
// instance can interact with both pins 0->15 or 32->47 at the same time." for
// the rp2350.
#define PIN_CLK 2
#define PIN_DATA 3
#define PIN_LATCH 4
#define PIN_BLANK 5
#define PIN_ROWS_BASE 6
#define N_ROWS 8
// Seems like 10MHz is around the limit for decently shaped pulses with my
// hardware. Have to do a proper rate test once everything is implemented.
#define TLC59283_TX_FREQ 500000

#define PIN_LED 25

int main() {
    stdio_init_all();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    gpio_init(PIN_LATCH);
    gpio_set_dir(PIN_LATCH, GPIO_OUT);

    gpio_init(PIN_BLANK);
    gpio_set_dir(PIN_BLANK, GPIO_OUT);
    gpio_put(PIN_BLANK, 0);  // Must be pulled low. Leaving it floating did cause problems (blank always asserted).

    for (int i = 0; i < N_ROWS; i++) {
        gpio_init(PIN_ROWS_BASE + i);
        gpio_set_dir(PIN_ROWS_BASE + i, GPIO_OUT);
        gpio_put(PIN_ROWS_BASE + i, 1);  // Initialize with all rows off (pins high)
    }

    // Prepare variables to hold references to used PIO, state-machine and PIO
    // program offset
    PIO pio = pio0;
    uint sm_tx = 0;

    // uint32_t test_pattern = 0b10101010101010101010101010101010;
    // uint32_t test_pattern = 0b10000000000000001000000000000000;
    uint16_t test_pattern = 0b1100110011001100;

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
    //bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&tlc59283_tx_program, &pio, &sm, &offset, PIN_CLK, 1, true);
    //hard_assert(success);
    uint offset_tx = pio_add_program(pio, &tlc59283_tx_program);

    tlc59283_tx_program_init(pio, sm_tx, offset_tx, PIN_CLK, PIN_DATA, TLC59283_TX_FREQ);

    sleep_ms(10);

    uint8_t current_row = 0;
    uint8_t next_row = current_row + 1;

    while(1) {
        gpio_put(PIN_LED, 0);
        // puts("Hello World\n");

        pio_sm_put_blocking(pio, sm_tx, test_pattern << 16);
        pio_sm_put_blocking(pio, sm_tx, test_pattern << 16);
        sleep_us(1);

        // Latch the shifted in value. Later we want to do this using an interrupt.
        gpio_put(PIN_BLANK, 1); // Blank the display
        gpio_put(PIN_LATCH, 1);
        sleep_us(1);
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


        // Rotate the pattern
        uint16_t x = test_pattern, n = 1;
        test_pattern = (x << n) | (x >> (16 - n));

        gpio_put(PIN_LED, 1);
        sleep_ms(1);
    }

    // This will free resources and unload our program.
    // Technically the program never reaches this point but keep the line here
    // so I don't forget about this function.
    pio_remove_program_and_unclaim_sm(&tlc59283_tx_program, pio, sm_tx, offset_tx);
}
