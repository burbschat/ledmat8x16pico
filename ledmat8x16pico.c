#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "tlc59283.pio.h"
#include <stdio.h>
#include <string.h>

// Pins for tlc59283 serial link.
// Both pins must be on the same GPIO group (0->31 or 16->47) as "No single PIO
// instance can interact with both pins 0->15 or 32->47 at the same time." for
// the rp2350.
#define PIN_CLK 2
#define PIN_DATA 3

// Pins for other tlc59283 controls (latch, blank)
#define PIN_LATCH 4
#define PIN_BLANK 5

// Row strobe pins
#define N_ROWS 8
#define PIN_ROWS_BASE 6 // Use N_ROWs row strobe pins starting at this pin

// Number of display modules for which data should be transmitted. It is possible to have less
// modules connected than this number. In this case the data is simply shifted past the last module
// (which ofc means that more data is transmitted for every row than strictly required).
#define N_DISPLAY_MODULES 7

// Bit rate for tlc59283 serial link.
// We transmit one bit per clock so this is equivalent to the serial links clock frequency.
// Seems like 10MHz is around the limit for decently shaped pulses with my hardware.
#define TLC59283_TX_FREQ 1000000

// Frame receive UART parameters.
// Could use 16 data bits as this corresponds to exactly the module width. Two transmissions is also
// fine. Keep 8 for now as this seems to be a more standard setting. Possibly required to adjust DMA
// settings if number of data bits is changed.
#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_BAUD_RATE 115200
#define UART_DATA_BITS 8
#define UART_PARITY UART_PARITY_NONE
#define UART_STOP_BITS 1

// LED Pin on the Pico board.
// Can be flashed to perhaps indicate state machine status or similar.
#define PIN_LED 25

// Some test patterns which may be used for development/tests.
// Bits are shifted right to left as indicated below.
//                            left most module    right most module <-> Pico
//      <-- shift direction   left       right    left       right   <-- shift direction
uint16_t test_pattern[2] = {0b1010101010101010, 0b1101000000000111};

// Frame buffer containing currently displayed frame and row length data.
// Row length data is stored in the first column and used by the PIO program to determine the number
// of bits to transmit. As we use 16 bit integers here, technically this limits the firmware to a
// maximum of 4096 chained modules (i.e. probably more than you will ever need to use).
volatile uint16_t frame_buffer[N_ROWS][N_DISPLAY_MODULES + 1];

// Frame buffer to be filled via UART, then latched into the actual frame buffer.
// Received frame data is written to this memory region via DMA, thus assuming that we have one
// continous region (rows appended to each other).
volatile uint16_t frame_buffer_uart_rx[N_ROWS][N_DISPLAY_MODULES];

// Prepare variables to hold references to used PIO, state-machine and PIO
// program offset. Use a variable instead of a macro as technically one can decide which PIO and sm
// to use at runtime (which I might do at some point).
PIO pio = pio0;
uint sm_tx = 0;

// Cariables to hold references to used DMA channels
int piodma_chan;
int uartdma_chan;

/**
 * @brief Interleave two 8 bit integers
 *
 * @param x Used for odd position
 * @param y Used for even positions
 * @return Interleaved bits of x and y as 16 bit integer
 */
uint16_t interleave_bits(uint8_t x, uint8_t y) {
    uint16_t z;
    // Shamelessly stolen from
    // http://graphics.stanford.edu/~seander/bithacks.html#Interleave64bitOps
    z = ((x * 0x0101010101010101ULL & 0x8040201008040201ULL) * 0x0102040810204081ULL >> 49) &
            0x5555 |
        ((y * 0x0101010101010101ULL & 0x8040201008040201ULL) * 0x0102040810204081ULL >> 48) &
            0xAAAA;
    return z;
}

/**
 * @brief Insert 64 bit integer representation of module frame into currently displayed frame buffer
 *
 * @param hex 64 bit representation of the module frame
 * @param n_module Module position in the frame buffer
 * @param r If true, red LEDs will be illuminated
 * @param g If true, green LEDs will be illuminated
 */
void frame_buffer_insert_hex(uint64_t hex, int n_module, bool r, bool g) {
    // May be used with output from this: https://xantorohara.github.io/led-matrix-editor/
    for (int row = 0; row < N_ROWS; row++) {
        uint8_t x = 0;
        uint8_t y = 0;
        if (g) {
            x = *((uint8_t *)&hex + row);
        }
        if (r) {
            y = *((uint8_t *)&hex + row);
        }

        frame_buffer[row][n_module + 1] = interleave_bits(x, y);
    }
}

void set_row_length(uint16_t length) {
    // Set the row length entries in the frame buffer (all the same value)
    for (int row = 0; row < N_ROWS; row++) {
        // First column actually contains the counter preset values, which are the row length - 1
        frame_buffer[row][0] = length - 1;
    }
}

/**
 * @brief Display hard coded default frame
 */
void display_default_frame() {
    for (int module = 0; module < N_DISPLAY_MODULES; module++) {
        // Display test pattern
        // for (int row = 0; row < N_ROWS; row++) {
        //     frame_buffer[row][module + 1] = test_pattern[module % 2];
        // }

        // Display some letters (hard coded using 64 bit representation)
        frame_buffer_insert_hex(0xc6c6e6f6decec600, 0, 1, 0);
        frame_buffer_insert_hex(0x6666667e66663c00, 1, 1, 1);
        frame_buffer_insert_hex(0x3c66760606663c00, 2, 0, 1);
        frame_buffer_insert_hex(0x3c66666666663c00, 3, 1, 1);
        frame_buffer_insert_hex(0x1818183c66666600, 4, 1, 0);
        frame_buffer_insert_hex(0x6666667e66663c00, 5, 1, 1);
        frame_buffer_insert_hex(0x0, 6, 0, 1);
    }
}

/**
 * @brief Initialize the currently displayed frame buffer
 */
void init_frame_buffer() {
    // Populate row length headers in frame buffer. They are all the same number, but to ease
    // passing the data via dma we duplicate the header for each row.
    set_row_length(N_DISPLAY_MODULES * 16);
    display_default_frame();
}

/**
 * @brief Rotate currently displayed frame buffer (to the left)
 *
 * @param n Number of positions to rotate
 */
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

bool modify_frame_callback(__unused struct repeating_timer *t) {
    rotate_frame_buffer(2); // Rotate two positions (one LED slot as there are two LEDs per slot)
    return true;
}

/**
 * @brief Handler called after each row transmission
 */
void __not_in_flash_func(row_done_handler)() {
    // Set as not in flash just in case, does not seem to make much of a difference though.
    // TODO: This function need some rework

    static bool first_run = true;

    static uint8_t current_row = 0;
    static uint8_t prev_row = N_ROWS - 1;

    if (first_run) {
        first_run = false;
    } else {
        gpio_put(PIN_BLANK, 1); // Blank the display
        // Strobe latch pin to latch shifted in value
        gpio_put(PIN_LATCH, 1);
        gpio_put(PIN_LATCH, 0);

        // Turn off previous row
        gpio_put(PIN_ROWS_BASE + prev_row, 1);
        // Turn on current row
        gpio_put(PIN_ROWS_BASE + current_row, 0);
        gpio_put(PIN_BLANK, 0); // Un-blank the display

        // Wait a little to set how long each row is displayed.
        // Must use busy_wait_ms (sleep_ms cannot be used in interrupt handler)
        // Pulsing rows too fast seems to result in very annoying audible noise from the LED module.
        // 5 ms per row seems to mitigate this to a tolerable level (lower frequency).
        // Not exactly sure if this is caused by the LEDs or the MOSFETs driving the rows.
        // Disconnecting LEDs causes the noise to vanish, but a MOSFET without load may as well be
        // quiet. Have to try with some different load (e.g. resistors).
        busy_wait_ms(1);

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

    // Dispatch DMA with the next data
    dma_channel_set_read_addr(piodma_chan, frame_buffer[current_row], true);

    // Clear PIO interrupt (and NVIC)
    // Apparently one must clear both in exactly this order.
    pio_interrupt_clear(pio, 0);
    irq_clear(PIO0_IRQ_0);
}

/**
 * @brief Handler called after complete frame received via UART
 */
void __not_in_flash_func(frame_received_handler)() {
    // Latch rx buffer to actual frame buffer
    for (int row = 0; row < N_ROWS; row++) {
        for (int col = 0; col < N_DISPLAY_MODULES; col++) {
            frame_buffer[row][col + 1] = frame_buffer_uart_rx[row][col];
        }
    }
    // Clear the interrupt
    dma_hw->ints0 = 1u << uartdma_chan;
    dma_channel_set_write_addr(uartdma_chan, frame_buffer_uart_rx, true);
}

void init_uart_frame_receive() {
    // Setup used pins
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Initialize UART for receiving frame data
    uart_init(UART_ID, UART_BAUD_RATE);
    // Set our data format
    uart_set_format(UART_ID, UART_DATA_BITS, UART_STOP_BITS, UART_PARITY);

    // Setup DMA for use with UART
    uartdma_chan = dma_claim_unused_channel(true);
    dma_channel_config uartdma_conf = dma_channel_get_default_config(uartdma_chan);
    // One module has 16 columns but uart characters are 8 bit, so transfer in 8 bit chunks
    channel_config_set_transfer_data_size(&uartdma_conf, DMA_SIZE_8);
    // Keep reading from the same address for each block (technically the default?)
    channel_config_set_read_increment(&uartdma_conf, false);
    // Increment write address after each written block
    channel_config_set_write_increment(&uartdma_conf, true);
    // Get data from UART RX
    channel_config_set_dreq(&uartdma_conf, UART_DREQ_NUM(UART_ID, false));

    dma_channel_configure(
        uartdma_chan, &uartdma_conf,
        frame_buffer_uart_rx,           // Don't provide a write address yet
        &((uart_hw_t *)UART_ID)->dr,    // Read address
        N_DISPLAY_MODULES * 2 * N_ROWS, // Receive one full frame at a time in 8 bit chunks
        true                            // Immediately start
    );

    // Enable interrupt via IRQ1 (IRQ0 used for PIO interrupt)
    dma_channel_set_irq1_enabled(uartdma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, frame_received_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

void init_tlc59283_interface() {
    // Setup used pins
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

    // Setup DMA
    piodma_chan = dma_claim_unused_channel(true);
    dma_channel_config piodma_conf = dma_channel_get_default_config(piodma_chan);
    channel_config_set_transfer_data_size(&piodma_conf, DMA_SIZE_16); // One module has 16 columns
    // Increment read address after each written block
    channel_config_set_read_increment(&piodma_conf, true);
    // Keep writing to the same address for each block (technically the default?)
    channel_config_set_write_increment(&piodma_conf, false);
    // Use data request signal from PIO0 TX FIFO (must be matched to used pio!)
    channel_config_set_dreq(&piodma_conf, DREQ_PIO0_TX0);

    dma_channel_configure(
        piodma_chan, &piodma_conf,
        &pio0_hw->txf[0],      // Write address (only need to set this once)
        NULL,                  // Don't provide a read address yet
        N_DISPLAY_MODULES + 1, // Write the same value many times, then halt and interrupt
        false                  // Don't start yet
    );

    // Add PIO program to the used PIO
    uint offset_tx = pio_add_program(pio, &tlc59283_tx_program);
    // Have irq trigger an interrupt
    // pis_interrupt0 SUPPOSEDLY! referes to PIO interrupt 0 (the one we use)
    pio_set_irq0_source_enabled(
        pio,
        (enum pio_interrupt_source)((uint)pis_interrupt0 + sm_tx), // Typecast gymnastics required
        true);
    // Setup handler to be called on pio interrupt.
    // TODO: Unique handler should actually be fine?
    irq_add_shared_handler(PIO0_IRQ_0, row_done_handler, 0);
    // Clear interrupt nr zero
    pio_interrupt_clear(pio, 0);
    irq_clear(PIO0_IRQ_0);
    // Enable PIO IRQ interrupt
    irq_set_enabled(PIO0_IRQ_0, true);

    // The callback is called immediately after loading the program, initializing the first DMA
    // transfer as the IRQ flag is set (and waited for to be cleared) at the top of the PIO program.
    tlc59283_tx_program_init(pio, sm_tx, offset_tx, PIN_CLK, PIN_DATA, TLC59283_TX_FREQ);

    // If we ever want to remove the program again, use something like the following:
    // pio_remove_program_and_unclaim_sm(&tlc59283_tx_program, pio, sm_tx, offset_tx);
}

int main() {
    stdio_init_all();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Initialize frame buffer
    init_frame_buffer();

    // Initialize tlc59283 interface
    init_tlc59283_interface();

    // Initialize UART for receiving frames
    init_uart_frame_receive();

    sleep_ms(10); // TODO: Do we really need this?

    // Setup timer for frame modification callback (do something like rotate the frame buffer)
    struct repeating_timer timer;
    add_repeating_timer_ms(50, modify_frame_callback, NULL, &timer);

    // Everything else from this point is interrupt-driven. The processor has
    // time to sit and think about its early retirement -- maybe open a bakery?
    while (true) tight_loop_contents();
}
