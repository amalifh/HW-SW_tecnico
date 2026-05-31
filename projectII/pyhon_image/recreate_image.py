from PIL import Image
import numpy as np

WIDTH = 512
HEIGHT = 512

input_bin = "output.bin"
output_image = "output.png"

arr = np.fromfile(input_bin, dtype=np.uint8)

expected_size = WIDTH * HEIGHT

if arr.size != expected_size:
    raise ValueError(f"Expected {expected_size} bytes, got {arr.size}")

arr = arr.reshape((HEIGHT, WIDTH))

img = Image.fromarray(arr, mode="L")
img.save(output_image)

print(f"Saved {output_image}")