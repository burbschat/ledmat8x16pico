#include "util.h"
#include "parameters.h"

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
