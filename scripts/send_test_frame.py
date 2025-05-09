import serial
import struct
import numpy as np
import time


# Must match parameters of the pico firmware!
N_DISPLAY_MODULES = 7
N_ROWS = 8


def frame_buffer_insert_hex(frame, hex: np.uint64, n_module: int, r: bool, g: bool):
    for row in range(N_ROWS):
        x = np.uint8(0)
        y = np.uint8(0)
        if g:
            x = np.uint8(hex >> row * 8)
        if r:
            y = np.uint8(hex >> row * 8)

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
        frame[row][n_module] = z


def transmit_frame(ser, frame):
    bytes_data = frame.tobytes()
    ser.write(bytes_data)


def main():
    # Setup serial interface (adjust port as required)
    ser = serial.Serial("/dev/ttyACM0")
    ser.baudrate = 115200
    ser.timeout = 1.0

    # Prepare local frame buffer array
    frame = np.zeros((N_ROWS, N_DISPLAY_MODULES), dtype=np.uint16)

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


if __name__ == "__main__":
    main()
