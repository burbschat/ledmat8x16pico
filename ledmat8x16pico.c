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
#define TLC59283_TX_FREQ 800

const uint LED_PIN = 25;
const uint MOSFET_TEST = 15;

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(MOSFET_TEST);
    gpio_set_dir(MOSFET_TEST, GPIO_OUT);

    // Prepare variables to hold references to used PIO, state-machine and PIO
    // program offset
    PIO pio;
    uint sm;
    uint offset;

    uint32_t test_pattern = 0b10101010101010101010101010101010;

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
    // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&tlc59283_tx_program, &pio, &sm, &offset, PIN_CLK, 1, true);
    hard_assert(success);

    tlc59283_tx_program_init(pio, sm, offset, PIN_CLK, PIN_DATA, TLC59283_TX_FREQ);

    while(1) {
        gpio_put(LED_PIN, 0);
        gpio_put(MOSFET_TEST, 0);
        sleep_ms(1000);
        gpio_put(LED_PIN, 1);
        gpio_put(MOSFET_TEST, 1);
        // puts("Hello World\n");
        pio_sm_put_blocking(pio, sm, test_pattern);
        sleep_ms(1000);
    }

    // This will free resources and unload our program.
    // Technically the program never reaches this point but keep the line here
    // so I don't forget about this function.
    pio_remove_program_and_unclaim_sm(&tlc59283_tx_program, pio, sm, offset);
}
