#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/rand.h"
#include "parameters.h"
#include "pico/stdlib.h"
#include "tlc59283.pio.h"
#include "util.h"
#include "tusb.h"
#include "bsp/board.h"
#include "pico/multicore.h"
#include <string.h>

// Some test patterns which may be used for development/tests and also illustrates how the data is
// arranged). Bits are shifted right to left as indicated below.
//                            left most module    right most module <-> Pico
//      <-- shift direction   left       right    left       right   <-- shift direction
uint16_t test_pattern[2] = {0b1010101010101010, 0b1101000000000111};

// Frame buffer containing some frames so we don't have to UART transmit on every refresh (too slow
// for larger displays).
// The actually displayed frame will be latched to a different buffer by some reoccurring handler
// (timer based or perhaps initiated by a special UART message).
volatile framebuffer_t frame_buffer;
// Variable to store the currently displayed frame
// This includes a 16 bit header to let the state machine know the length of each row. We cannot
// hard-code the row length in the state machine code as there the largest number one can assign to
// a register is 31. The way around this is to read the number from the RX fifo, which is what we do
// here. These header values are not touched once the frame buffer has been initialized.
// As we use 16 bit integers here, technically this limits the firmware to a maximum of 4096 chained
// modules (i.e. probably more than you will ever need to use).
volatile h_frame_t current_frame_buffer;
volatile uint32_t current_frame_number;
volatile uint32_t current_frame_offset;

// Frame buffer to be filled via UART, then latched into the actual frame buffer.
// Received frame data is written to this memory region via DMA, thus assuming that we have one
// continous region (rows appended to each other). This does not include a 16 bit row length
// header.
volatile r_frame_t frame_buffer_uart_rx;

// Keep track of the last row pulsed an next one for which data should be transmitted.
volatile uint8_t current_row = N_ROWS - 1;
volatile uint8_t next_row = 0;

// Prepare variables to hold references to used PIO, state-machine and PIO
// program offset. Use a variable instead of a macro as technically one can decide which PIO and sm
// to use at runtime (which I might do at some point).
PIO pio = pio0;
uint sm_tx = 0;

// Variables to hold references to used DMA channels
int piodma_chan;
int uartdma_chan;

void set_row_length(uint16_t length) {
    // Set the row length entries in the current frame buffer
    for (int row = 0; row < N_ROWS; row++) {
        // First column contains the counter preset values, which are the row length - 1
        current_frame_buffer[row][0] = length - 1;
    }
}

/**
 * @brief Insert 64 bit integer representation of module frame into a frame
 *
 * @param frame Pointer to frame in the frame buffer (including row headers)
 * @param hex 64 bit representation of the module frame
 * @param n_module Module position in the frame buffer
 * @param r If true, red LEDs will be illuminated
 * @param g If true, green LEDs will be illuminated
 */
void frame_insert_modulewise(int frame_number, uint64_t hex, int n_module, bool r, bool g) {
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

        frame_buffer[frame_number][row][n_module] = interleave_bits(x, y);
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
        frame_insert_modulewise(0, 0xc6c6e6f6decec600, 0, 1, 0);
        frame_insert_modulewise(0, 0x6666667e66663c00, 1, 1, 1);
        frame_insert_modulewise(0, 0x3c66760606663c00, 2, 0, 1);
        // frame_insert_modulewise(0, 0x3c66666666663c00, 3, 1, 1);
        // frame_insert_modulewise(0, 0x1818183c66666600, 4, 1, 0);
        // frame_insert_modulewise(0, 0x6666667e66663c00, 5, 1, 1);
        // frame_buffer_insert_hex(0x0, 6, 0, 1);
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
    // For now just rotate the currently displayed frame.
    // Later implement this as a constant offset applied when reading a frame from the frame buffer
    // to the currently displayed frame buffer.
    for (int row = 0; row < N_ROWS; row++) {
        // Left most bits of left most module moved to right side of module
        uint16_t carry = current_frame_buffer[row][0 + 1] << (16 - n);
        uint16_t next_carry;
        for (int module = N_DISPLAY_MODULES - 1; module >= 0;
             module--) { // Start with right most module
            // Left most bit of current module (to be shifted out) moved to right side of module
            next_carry = current_frame_buffer[row][module + 1] << (16 - n);
            current_frame_buffer[row][module + 1] = (current_frame_buffer[row][module + 1] >> n) | carry;
            carry = next_carry; // Update carry for next module to the shifted out bit
        }
    }
}

/**
 * @brief Copy frame from frame buffer to currently displayed frame buffer
 *
 * @param frame_number Position of the fram in the frame buffer
 * @param offset Offset to apply to the frame (shift to the left)
 */
void latch_frame(uint32_t frame_number, uint32_t offset) {
    int32_t bit_offset = offset % 16;
    int32_t module_offset = offset / 16;
    for (int row = 0; row < N_ROWS; row++) {
        // Left most bits of left most module moved to right side of module
        uint16_t carry = frame_buffer[frame_number][row][module_offset % N_DISPLAY_MODULES] << (16 - bit_offset);
        uint16_t next_carry;
        for (int module = N_DISPLAY_MODULES - 1; module >= 0; module--) { // Start with right most module
            // Left most bit of current module (to be shifted out) moved to right side of module
            next_carry = frame_buffer[frame_number][row][(module + module_offset) % N_DISPLAY_MODULES] << (16 - bit_offset);
            current_frame_buffer[row][module + 1] = (frame_buffer[frame_number][row][(module + module_offset) % N_DISPLAY_MODULES] >> bit_offset) | carry;
            carry = next_carry; // Update carry for next module to the shifted out bit
        }
    }
    current_frame_number = frame_number;
    current_frame_offset = offset;
}

bool update_frame_callback(__unused struct repeating_timer *t) {
    // Rotate to the left
    latch_frame((current_frame_number + 1) % FB_DEPTH, current_frame_offset % (N_DISPLAY_MODULES * 16));
    // Rotate to the right
    // latch_frame(current_frame_number, (current_frame_offset + N_DISPLAY_MODULES * 16 - 2) % (N_DISPLAY_MODULES * 16));
    // rotate_frame_buffer(2); // Rotate two positions (one LED slot as there are two LEDs per slot)
    return true;
}

bool clear_row_done_interrupt(__unused struct repeating_timer *t) {
    // Dispatch DMA with the next data.
    // The state machine will stay stalled until this call as the input fifo should be empty.
    dma_channel_set_read_addr(piodma_chan, current_frame_buffer[next_row], true);
    // Return false to remove the timer that called this callback (i.e. operate as a one-shot timer)
    return false;
}

bool row_blank_interrupt(__unused struct repeating_timer *t) {
    // Add another timer and blank the display for some time before sending the next row.
    // This can be used as a way to adjust perceived brightness. It should be hooked into the row
    // update cycle to avoid beating due to different frequencies of blank pulsing and row update.
    static struct repeating_timer row_blank_timer;
    gpio_put(PIN_BLANK, 1); // Blank the display
    uint32_t row_clear_us = ROW_CLEAR_US;
    #ifdef TIMER_RANDOM_OFFSET
    uint16_t random_offset = get_rand_32() & ~(~0u << TIMER_RANDOM_OFFSET);
    row_clear_us = row_clear_us + random_offset;
    #endif /* ifdef TIMER_RANDOM_OFFSET */
    add_repeating_timer_us(row_clear_us, clear_row_done_interrupt, NULL, &row_blank_timer);
    return false;
}

/**
 * @brief Handler called after each row transmission
 */
void __not_in_flash_func(row_done_handler)() {
    // Set as not in flash just in case, does not seem to make much of a difference though.
    // TODO: This function need some rework

    static bool first_run = true;

    static struct repeating_timer row_illuminated_timer;

    if (first_run) {
        first_run = false;
    } else {
        // Here the display is already blank as we set it blank as the second step of the wait until
        // the next row transmit. 
        // Strobe latch pin to latch shifted in value
        gpio_put(PIN_LATCH, 1);
        // This cannot be to short of a pulse! Maybe include the blank/latch pulse in PIO program?
        // As the shortest wait function (busy_wait_us) is us, use nop instruction instead (scales
        // with clock but at 125MHz we should have 8ns). For more delay add more nops with new lines
        // like "nop\nnop\nnop".
        __asm volatile("nop" :);
        gpio_put(PIN_LATCH, 0);

        // Turn off current row
        gpio_put(ROW_GPIO_FW(current_row), 1);
        // Turn on next row
        gpio_put(ROW_GPIO_FW(next_row), 0);
        gpio_put(PIN_BLANK, 0); // Un-blank the display

        // Make the current row the previously next and set new next row
        current_row = next_row;
        next_row = (current_row + 1) % 8;
    }

    // Clear PIO interrupt (and NVIC).
    // Apparently one must clear both in exactly this order as the pio interrupt set seems to cause
    // the NVIC interrupt to also be asserted.
    // Interrupts are reset but PIO will stay stalled until dma feeds in data!
    pio_interrupt_clear(pio, 0);
    irq_clear(PIO0_IRQ_0);

    // Set a timer and call a callback that dispatches the dma once the timer expires. This way we
    // can make sure to wait more or less the time we want the row to be illuminated without
    // blocking the whole core. Have the interrupts return false, which will cause the timer to be
    // removed on return of the callback.
    // The interrupt flags must be cleared before returning from this function as if not cleared
    // this handler is just immediately called again. Thus we cannot have the PIO SM wait for the
    // IRQ flag to be cleared. Instead we utilize that the SM will stay stalled at the out
    // instruction until there is some data in the RX fifo (provided via DMA). The DMA then is
    // dispatched by the callback called by the timer.
    // Actually there are two timers, the first one for the duration the row is on, the second one
    // for the duration it will be off (just assert blank pin). This can be used as a way of
    // reducing the display brighness and appears to work pretty good.
    uint32_t row_illuminate_us = ROW_ILLUMINATE_US;
    #ifdef TIMER_RANDOM_OFFSET
    uint16_t random_offset = get_rand_32() & ~(~0u << TIMER_RANDOM_OFFSET);
    row_illuminate_us = row_illuminate_us + random_offset;
    #endif /* ifdef TIMER_RANDOM_OFFSET */
    add_repeating_timer_us(row_illuminate_us, row_blank_interrupt, NULL, &row_illuminated_timer);
}


#ifdef FRAME_RECEIVE_UART
/**
 * @brief Handler called after complete frame received via UART
 */
void __not_in_flash_func(frame_received_handler)() {
    // If invalid frame index received, ignore the received data
    if (frame_buffer_uart_rx.frame_i < FB_DEPTH) {
        // Latch rx buffer to actual frame buffer
        for (int row = 0; row < N_ROWS; row++) {
            for (int col = 0; col < N_DISPLAY_MODULES; col++) {
                // Write to selected frame in the frame buffer
                frame_buffer[frame_buffer_uart_rx.frame_i][row][col] = frame_buffer_uart_rx.frame[row][col];
            }
        }
    }
    // Clear the interrupt
    dma_hw->ints0 = 1u << uartdma_chan;
    dma_channel_set_write_addr(uartdma_chan, &frame_buffer_uart_rx, true);
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
    channel_config_set_transfer_data_size(&uartdma_conf, UART_DMA_SIZE);
    // Keep reading from the same address for each block (technically the default?)
    channel_config_set_read_increment(&uartdma_conf, false);
    // Increment write address after each written block
    channel_config_set_write_increment(&uartdma_conf, true);
    // Get data from UART RX
    channel_config_set_dreq(&uartdma_conf, UART_DREQ_NUM(UART_ID, false));

    dma_channel_configure(
        uartdma_chan, &uartdma_conf,
        &frame_buffer_uart_rx,                  // Don't provide a write address yet
        &((uart_hw_t *)UART_ID)->dr,            // Read address
        sizeof(r_frame_t) * 8 / UART_DATA_BITS, // Receive one full frame at a time in 8 bit chunks
        true                                    // Immediately start
    );

    // Enable interrupt via IRQ1 (IRQ0 used for PIO interrupt)
    dma_channel_set_irq1_enabled(uartdma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, frame_received_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}
#endif

#ifdef FRAME_RECEIVE_TINYUSB
// Function handling TinyUSB to be run on second core
void tinyusb_main() {

    board_init();  // Board init required for USB

    tusb_init(); // Initialize TinyUSB

    while (true) {
        tud_task(); // TinyUSB device task loop

        // Handle CDC data
        if (tud_cdc_available()) {
            uint8_t buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf));

            // Echo back
            tud_cdc_write(buf, count);
            tud_cdc_write_flush();
        }
    }
}

void init_tinyusb_frame_receive() {
    // Run on second core as TinyUSB does not support proper interrupts
    multicore_launch_core1(tinyusb_main);
}
#endif

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

    // Initialize UART/TinyUSB for receiving frames
    #ifdef FRAME_RECEIVE_UART
    init_uart_frame_receive();
    #endif
    #ifdef FRAME_RECEIVE_TINYUSB
    init_tinyusb_frame_receive();
    #endif

    // Setup timer for frame modification callback (do something like rotate the frame buffer)
    struct repeating_timer timer;
    add_repeating_timer_ms(50, update_frame_callback, NULL, &timer);

    // Everything else from this point is interrupt-driven. The processor has
    // time to sit and think about its early retirement -- maybe open a bakery?
    while (true) tight_loop_contents();
}
