# Convert an image to a pixelated representation using pyxelate

from skimage import io
from pyxelate import Pyx, Pal

# Read image of Tux from the interwebs.
# Does look a lot bettern than expected even if downscaled to 8x8 and three colors.
image = io.imread("https://upload.wikimedia.org/wikipedia/commons/thumb/3/35/Tux.svg/506px-Tux.svg.png")

# Define a three color palette with red green and orangeish
led_pal = Pal.from_rgb([[255, 0, 0], [0, 255, 0], [255, 128, 0]])

# Instantiate Pyx transformer
# Tiny image will ofc. not look very good
pyx = Pyx(height=8, width=8, palette=led_pal)

# Fit the image to learn palette
pyx.fit(image)

# Transform input image
new_image = pyx.transform(image)

# Save the output
io.imsave("example_image.png", new_image)
