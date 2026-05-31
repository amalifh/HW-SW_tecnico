from PIL import Image
import numpy as np

WIDTH = 512
HEIGHT = 512

input_image = "einstein.png"
output_bin = "input.bin"

img = Image.open(input_image).convert("L")
img = img.resize((WIDTH, HEIGHT))

arr = np.array(img, dtype=np.uint8)

# Row-major uint8 binary
arr.tofile(output_bin)

print(f"Saved {output_bin}")
print(f"Shape: {arr.shape}")
print(f"Bytes: {arr.nbytes}")