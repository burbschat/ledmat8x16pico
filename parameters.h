#include "pico/stdlib.h"
#ifndef LEDMAT_PARAMETERS
#define LEDMAT_PARAMETERS

// Pins for tlc59283 serial link.
// Both pins must be on the same GPIO group (0->31 or 16->47) as "No single PIO
// instance can interact with both pins 0->15 or 32->47 at the same time." for
// the rp2350.
#define PIN_CLK 3
#define PIN_DATA 2

// Pins for other tlc59283 controls (latch, blank)
#define PIN_LATCH 17
#define PIN_BLANK 16

// Row strobe pins
#define N_ROWS 8
#define PIN_ROWS_BASE 6 // Use N_ROWs row strobe pins starting at this pin
// Due to board layout constraints, the order actually ended up inverted. Use
// the following macro to always get the correct GPIO pin number from the row
// number. While on the board we use numbers 1 to 8, in firmware use number 0
// to 7 (makes more sense in code).
#define ROW_GPIO_FW(ROW) PIN_ROWS_BASE + N_ROWS - 1 - ROW
// Same but use numbers as used in hardware (1 to 8).
#define ROW_GPIO_HW(ROW) ROW_GPIO_FW(ROW - 1)

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
#define N_DISPLAY_MODULES 8

// Maximum number of frames the on-device frame buffer can hold
#define FB_DEPTH 32

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

// Select TinyUSB on second core or interrupt based UART on same core for frame receive.
// Both may be enabled at the same time, but simultaneous write is not supported, so in
// practice only one of the two should be used.
#define FRAME_RECEIVE_UART
#define FRAME_RECEIVE_TINYUSB

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
