from PIL import Image
import numpy as np

WIDTH = 1280
HEIGHT = 720

arr32 = np.fromfile("output.bin", dtype=np.uint32)

if arr32.size != WIDTH * HEIGHT:
    raise ValueError(f"Expected {WIDTH * HEIGHT} words, got {arr32.size}")

arr8 = (arr32 & 0xFF).astype(np.uint8)
arr8 = arr8.reshape((HEIGHT, WIDTH))

Image.fromarray(arr8, mode="L").save("fpga_output.png")
print("Saved fpga_output.png")