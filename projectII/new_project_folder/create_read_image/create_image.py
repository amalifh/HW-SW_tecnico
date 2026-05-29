from PIL import Image
import numpy as np

WIDTH = 1280
HEIGHT = 720

img = Image.open("tiger_grayscale_resized.png").convert("L").resize((WIDTH, HEIGHT))
img.save("input_gray.png")

arr8 = np.array(img, dtype=np.uint8).flatten(order="C")

arr8.tofile("input.bin")

print("input.bin written as uint8 pixels")
print("File size should be:", WIDTH * HEIGHT, "bytes")