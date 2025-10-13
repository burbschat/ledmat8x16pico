from skimage import io
import numpy as np
import time
from bdfparser import Font
from PIL import Image, ImageFont, ImageDraw

np.set_printoptions(edgeitems=100, linewidth=1000000, formatter=dict(float=lambda x: "%.3g" % x))
from send_test_frame import N_ROWS, N_DISPLAY_MODULES, transmit_frame, setup_serial

N_MODULE_ROWS = 2


def unsnake(arr):
    h, w = arr.shape
    # Check the shape
    assert w % 16 == 0
    assert N_DISPLAY_MODULES % (w // 16) == 0
    # Numbers of blocks in vertical (y) and horizontal (x) direction
    blocks_y = h // N_ROWS

    # Snake from top right to left and downwards
    blocks = []
    # Every other row must be rotated by 180 degrees to get the snake pattern.
    # rotate = 1  # Do not rotate the first block
    for block_row in reversed(range(blocks_y)):
        block = arr[block_row * N_ROWS : (block_row + 1) * 8, ::1]
        rotate = (-1) ** (block_row)
        block = block[::rotate, ::rotate]
        blocks.append(block)

    # Concatenate all blocks horizontally
    unsnaked = np.hstack(blocks)
    return unsnaked


def test_unsnake(module_rows=2):
    # Only makes sense if number of modules is a multiple of the number of block rows
    assert N_DISPLAY_MODULES % module_rows == 0
    shape = (module_rows * N_ROWS, N_DISPLAY_MODULES // module_rows * 16)
    frame = np.arange(N_ROWS * N_DISPLAY_MODULES * 16).reshape(shape)  # Unique numbers for testing
    print(frame)
    print(frame.shape)
    frame_unsnaked = unsnake(frame)
    print(frame_unsnaked)
    print(frame_unsnaked.shape)


def packbits_uint16_horizontal(bits):
    # Reshape into groups of 16 bits
    bit_groups = bits.reshape(bits.shape[0], -1, 16)
    # Invert the bit order in each group (LSB to MSB instead of MSB to LSB)
    # This must be matched to the firmware bit shifting pio behaviour
    bit_groups = bit_groups[..., ::-1]
    # Define weights for each bit position (MSB to LSB)
    weights = 2 ** np.arange(15, -1, -1, dtype=np.uint16)
    # weights = weights[::-1]
    # Dot product to get int16 representation
    packed = (bit_groups @ weights).astype(np.uint16)
    return packed


def pad_or_crop(arr, target_shape, pad_value=0, alignment="top,left"):
    h, w = arr.shape
    th, tw = target_shape

    def get_start_indices(size, target, align):
        if align in ("center", "middle"):
            start = max((size - target) // 2, 0)
        elif align in ("top", "left"):
            start = 0
        elif align in ("bottom", "right"):
            start = max(size - target, 0)
        else:
            raise ValueError(f"Invalid alignment: {align}")
        return start

    # Parse alignment string
    alignment = alignment.lower()
    if "," in alignment:
        vert_align, horiz_align = alignment.split(",")
    else:
        vert_align = horiz_align = alignment

    # Crop indices
    ch_start = get_start_indices(h, th, vert_align)
    cw_start = get_start_indices(w, tw, horiz_align)
    cropped = arr[ch_start : ch_start + min(h, th), cw_start : cw_start + min(w, tw)]

    # Pad amounts
    pad_h = max(th - cropped.shape[0], 0)
    pad_w = max(tw - cropped.shape[1], 0)

    def get_pad(amount, align):
        if align in ("center", "middle"):
            return (amount // 2, amount - amount // 2)
        elif align in ("top", "left"):
            return (0, amount)
        elif align in ("bottom", "right"):
            return (amount, 0)
        else:
            raise ValueError(f"Invalid alignment: {align}")

    pad_top, pad_bottom = get_pad(pad_h, vert_align)
    pad_left, pad_right = get_pad(pad_w, horiz_align)

    return np.pad(cropped, ((pad_top, pad_bottom), (pad_left, pad_right)), mode="constant", constant_values=pad_value)


def prepare_frame(red_pos, grn_pos, alignment="top,left"):
    assert N_DISPLAY_MODULES % N_MODULE_ROWS == 0
    target_shape = (N_MODULE_ROWS * 8, (N_DISPLAY_MODULES // N_MODULE_ROWS) * 8)
    red_pos_padcrop = pad_or_crop(red_pos, target_shape=target_shape, alignment=alignment)
    grn_pos_padcrop = pad_or_crop(grn_pos, target_shape=target_shape, alignment=alignment)

    red_pos_padcrop_unsnaked = unsnake(red_pos_padcrop)
    grn_pos_padcrop_unsnaked = unsnake(grn_pos_padcrop)

    # Interleave red and green
    interleaved = np.empty((red_pos_padcrop_unsnaked.shape[0], red_pos_padcrop_unsnaked.shape[1] * 2), dtype=np.bool)
    interleaved[:, ::2] = grn_pos_padcrop_unsnaked
    interleaved[:, 1::2] = red_pos_padcrop_unsnaked

    interleaved_packed = packbits_uint16_horizontal(interleaved)

    return interleaved_packed


def large_frame_image(image_path="./example_image_16x16.png"):
    ser = setup_serial()

    image = io.imread(image_path)

    # Check if image has an alpha channel and add one if it does not...
    if image.shape[-1] == 3:
        # Add fully opaque alpha channel
        alpha_channel = np.ones((image.shape[0], image.shape[1], 1), dtype=image.dtype) * 255
        image = np.concatenate((image, alpha_channel), axis=-1)

    # Some threshold above which a pixel is considered active
    threshold = 255 / 2
    # For some reason there may be color values where there is transparency.
    # To fix this take the and between transparency region and color regions.
    alpha_pos = image[:, :, 3] > threshold
    red_pos = (image[:, :, 0] > threshold) & alpha_pos
    grn_pos = (image[:, :, 1] > threshold) & alpha_pos

    # Test pattern
    # red_pos[:] = 0
    # grn_pos[:] = 0
    # n = 11
    # red_pos[:n, :n] = 1
    # grn_pos[n:, n:] = 1

    alignments = ["top,left", "top,center", "top,right", "bottom,right", "bottom,center", "bottom,left"]
    for i in range(100):
        ready_to_transmit_frame = prepare_frame(red_pos, grn_pos, alignment=alignments[i % len(alignments)])
        transmit_frame(ser, ready_to_transmit_frame, 0)
        time.sleep(0.05)

    time.sleep(ser.timeout)  # Wait a little before returning to ensure transmit complete


class FontTTF:
    def __init__(self, font_path):
        self.font = ImageFont.truetype(font_path, size=16)

    def get_char(self, char):
        bbox = self.font.getbbox(char)
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]

        # Create monochrome image with enough space
        image = Image.new("1", (width, height), 0)
        draw = ImageDraw.Draw(image)

        # Note: offset by (-bbox[0], -bbox[1]) to account for font origin
        draw.text((-bbox[0], -bbox[1]), char, font=self.font, fill=1)

        # Convert to numpy array
        return np.array(image, dtype=np.uint8)


class FontBDF:
    def __init__(self, font_path):
        self.font = Font(font_path)

    def get_char(self, char):
        return np.array(self.font.glyph(char).draw().todata(datatype=2))


def large_static_text():
    # Load bitmap font
    # font = FontBDF("./fonts/ibmfonts/bdf/ib16x16u.bdf")
    font = FontTTF("./fonts/jfdotfont/JF-Dot-jiskan16.ttf")

    ser = setup_serial()

    # Text to display
    # text = "Hello this is a long sentence!  "
    text = "/r次は/g東京/g　/r/g右側/g/rの扉が開きます　　"

    red_pos = None
    grn_pos = None

    state = "normal"
    en_red = False
    en_grn = False
    for char in text:
        if state == "normal":
            if char == "/":
                state = "command"
                continue
            else:
                next_char_data = font.get_char(char)
                if en_red:
                    next_char_data_red = next_char_data
                else:
                    next_char_data_red = np.zeros_like(next_char_data)

                if en_grn:
                    next_char_data_grn = next_char_data
                else:
                    next_char_data_grn = np.zeros_like(next_char_data)

                if grn_pos is not None:
                    grn_pos = np.hstack([grn_pos, next_char_data_grn])
                else:
                    grn_pos = next_char_data_grn

                if red_pos is not None:
                    red_pos = np.hstack([red_pos, next_char_data_red])
                else:
                    red_pos = next_char_data_red
        elif state == "command":
            if char == "r":
                en_red = not en_red
            if char == "g":
                en_grn = not en_grn
            state = "normal"
            continue

    # Shift each row to the left
    def rotate_frame(red_pos, grn_pos):
        red_pos = np.roll(red_pos, -1, axis=1)
        grn_pos = np.roll(grn_pos, -1, axis=1)
        return red_pos, grn_pos

    # For now this will only properly work (no sudden jumps) if text is padded to match frame buffer depth
    for i in range(32 * 16):
        ready_to_transmit_frame = prepare_frame(red_pos, grn_pos, alignment="left")
        transmit_frame(ser, ready_to_transmit_frame, i)
        # time.sleep(0.01)
        red_pos, grn_pos = rotate_frame(red_pos, grn_pos)

    # Apparenty not wating for a little here causes inclomplete transmission even when I set no timeout for the serial.
    # Write should be blocking, so not sure what is the problem here.
    time.sleep(ser.timeout)


def main():
    # large_frame_image()
    large_static_text()
    # test_unsnake()


if __name__ == "__main__":
    main()
