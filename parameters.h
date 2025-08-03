#include "pico/stdlib.h"
#ifndef LEDMAT_PARAMETERS
#define LEDMAT_PARAMETERS

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

// Time each row will be illuminated and not illuminated. The sum of those times controls the time
// per row update. Adjust the ratio to reduce perceived brightness.
#define ROW_ILLUMINATE_US 300
#define ROW_CLEAR_US 700

// Enable random offsest added to the timer to reduce annoying single frequency noise to some more
// broadband noise. Value is the number of bits of a 16bit random integer to be used for the offset.
// Thus the maximum value is 16. Realistically ~9 is probably the maximum value one would chose.
#define TIMER_RANDOM_OFFSET 8

// Number of display modules for which data should be transmitted. It is possible to have less
// modules connected than this number. In this case the data is simply shifted past the last module
// (which ofc means that more data is transmitted for every row than strictly required).
#define N_DISPLAY_MODULES 9

// Maximum number of frames the on-device frame buffer can hold
#define FB_DEPTH 5

// Bit rate for tlc59283 serial link.
// We transmit one bit per clock so this is equivalent to the serial links clock frequency.
// Was able to drive my modules at the advertised frequency of 35MHz for the tlc59283. This would
// work even with 40cm extension wires between modules on the first prototype which has some clock
// coupling into neighboring lines.
#define TLC59283_TX_FREQ 20000000

// Frame receive UART parameters.
// Could use 16 data bits as this corresponds to exactly the module width. Two transmissions is also
// fine. Keep 8 for now as this seems to be a more standard setting. Possibly required to adjust DMA
// settings if number of data bits is changed.
#define UART_ID uart0
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_BAUD_RATE 115200
#define UART_DATA_BITS 8
#define UART_DMA_SIZE DMA_SIZE_8 // Must match UART_DATA_BITS
#define UART_PARITY UART_PARITY_NONE
#define UART_STOP_BITS 1

// LED Pin on the Pico board.
// Can be flashed to perhaps indicate state machine status or similar.
#define PIN_LED 25

// Types for row, frame and framebuffer itself (rows without header)
typedef uint16_t row_t[N_DISPLAY_MODULES];
typedef row_t frame_t[N_ROWS];
typedef frame_t framebuffer_t[FB_DEPTH];
// Types for a frame with row header (used for currently displayed frame)
typedef uint16_t h_row_t[N_DISPLAY_MODULES + 1];
typedef h_row_t h_frame_t[N_ROWS];
// Types for UART frame transfer (with metadata header)
typedef struct r_frame {
    uint16_t frame_i; // Frame index (location in frame buffer)
    frame_t frame;    // The actual frame data
} r_frame_t;

#endif // !LEDMAT_PARAMETERS
