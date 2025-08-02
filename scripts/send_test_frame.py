import serial
import numpy as np
import time
from bdfparser import Font
from datetime import datetime
from skimage import io


# Must match parameters of the pico firmware!
N_DISPLAY_MODULES = 9
N_ROWS = 8

def interleave_uint8(x, y):
    # Cast just in case
    x = np.uint8(x)
    y = np.uint8(y)

    # Interleave bits
    # Shamelessly stolen from http://graphics.stanford.edu/~seander/bithacks.html#Interleave64bitOps
    # Ignore numpy overflow warnings as they always occur as a product of the following operation
    np.seterr(over="ignore")
    z = np.uint16(
        ((x * np.uint64(0x0101010101010101) & np.uint64(0x8040201008040201)) * np.uint64(0x0102040810204081) >> 49)
        & 0x5555
        | (
            (y * np.uint64(0x0101010101010101) & np.uint64(0x8040201008040201)) * np.uint64(0x0102040810204081)
            >> 48
        )
        & 0xAAAA
    )
    return z

def frame_buffer_insert_hex(frame, hex: np.uint64, n_module: int, r: bool, g: bool):
    for row in range(N_ROWS):
        x = np.uint8(0)
        y = np.uint8(0)
        if g:
            x = np.uint8(hex >> row * 8)
        if r:
            y = np.uint8(hex >> row * 8)

        frame[row][n_module] = interleave_uint8(x, y)


def frame_buffer_insert_single(frame, frame_single, n_module: int, r: bool, g: bool, op=None):
    # Insert non-interlaced frame (single color) into the frame buffer
    for row in range(N_ROWS):
        x = np.uint8(0)
        y = np.uint8(0)
        if g:
            x = frame_single[row]
        if r:
            y = frame_single[row]
        if op is None:
            # Overwrite with the new values
            frame[row][n_module] = interleave_uint8(x, y)
        elif op == "and":
            # Take and between new and present values
            frame[row][n_module] &= interleave_uint8(x, y)
        elif op == "or":
            # Take or between new and present values
            frame[row][n_module] |= interleave_uint8(x, y)
        else:
            raise ValueError("Operation must be 'and' or 'or' or None.")


def transmit_frame(ser, frame):
    bytes_data = frame.tobytes()
    ser.write(bytes_data)


def setup_serial(port="/dev/ttyACM0"):
    # Setup serial interface (adjust port as required)
    ser = serial.Serial(port)
    ser.baudrate = 115200
    ser.timeout = 1.0
    return ser

def get_empty_frame_buffer():
    # Prepare local frame buffer array
    return np.zeros((N_ROWS, N_DISPLAY_MODULES), dtype=np.uint16)

def example_static_frame():
    ser = setup_serial()

    frame = get_empty_frame_buffer()

    # Some hex value encoded frame data (1 value per module)
    hex_val = np.uint64(0x3C66760606663C00)
    # hex_val = np.uint64(0xff00ff00ff00ff00)

    # Insert some data into local frame buffer
    for i in range(N_DISPLAY_MODULES):
        frame_buffer_insert_hex(frame, hex_val, i, True, False)

    # Print the frame data
    # print(np.vectorize(np.binary_repr)(frame, width=16))

    # Transmit the frame
    transmit_frame(ser, frame)
    # Apparenty not wating for a little here causes inclomplete transmission even when I set no timeout for the serial.
    # Write should be blocking, so not sure what is the problem here.
    time.sleep(ser.timeout)


def get_char_data(font, char):
    char_data = font.glyph(char).draw().todata()
    # Convert to actually numbers (from strings)
    # Must do some slicing to invert bit order
    char_data = [np.uint8(int(char[::-1], 2)) for char in char_data]
    return char_data


def example_static_text():
    # Load bitmap font
    font = Font("./fonts/ibmfonts/bdf/ib8x8u.bdf")

    # Text to display
    text = "123456789"

    ser = setup_serial()

    frame = get_empty_frame_buffer()

    for i in range(N_DISPLAY_MODULES):
        char_data = get_char_data(font, text[i % len(text)])
        frame_buffer_insert_single(frame, char_data, i, False, True)

    # print(np.vectorize(np.binary_repr)(frame, width=16))

    # Transmit the frame
    transmit_frame(ser, frame)
    # Apparenty not wating for a little here causes inclomplete transmission even when I set no timeout for the serial.
    # Write should be blocking, so not sure what is the problem here.
    time.sleep(ser.timeout)


def example_time():
    # Load bitmap font
    font = Font("./fonts/ibmfonts/bdf/ib8x8u.bdf")

    ser = setup_serial()

    frame = get_empty_frame_buffer()

    while True:
        text = datetime.now().strftime("%H:%M:%S ")
        # text = datetime.now().strftime(" %S ")
        # text = str(time.time_ns())[-3:]

        for i in range(N_DISPLAY_MODULES):
            char_data = get_char_data(font, text[i % len(text)])
            frame_buffer_insert_single(frame, char_data, i, False, True)

        # print(np.vectorize(np.binary_repr)(frame, width=16))

        # Transmit the frame
        transmit_frame(ser, frame)
        time.sleep(1/60)

def example_image():
    # Setup UART serial
    ser = setup_serial()

    # Tiny 8x8 image
    image = io.imread("./example_image.png")

    # Some threshold above which a pixel is considered active
    threshold = 255 / 2
    # For some reason there may be color values where there is transparency.
    # To fix this take the and between transparency region and color regions.
    alpha_pos = (image[:, :, 3] > threshold)
    red_pos = (image[:, :, 0] > threshold) & alpha_pos
    grn_pos = (image[:, :, 1] > threshold) & alpha_pos

    # Convert to 8 bit integers
    red_pos = np.packbits(red_pos)
    grn_pos = np.packbits(grn_pos)

    # Insert the image into the frame buffer
    frame = get_empty_frame_buffer()

    for i in range(N_DISPLAY_MODULES):
        frame_buffer_insert_single(frame, red_pos, i, True, False)
        frame_buffer_insert_single(frame, grn_pos, i, False, True, op="or")

    # Transmit the frame
    transmit_frame(ser, frame)
    time.sleep(ser.timeout)


if __name__ == "__main__":
    example_static_text()
