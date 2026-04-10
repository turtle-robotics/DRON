from PIL import Image
import glob
import os

image_paths = glob.glob(r"C:/Users/aidan/Downloads/test flight/imgs/butt/left/*.jpg")+ glob.glob(r"C:/Users/aidan/Downloads/test flight/imgs/butt/right/*.jpg")
images = []

thumb_width = 120
thumb_height = 100
cols = 30 # 20 images per row


for path in image_paths:
    if os.path.exists(path):
        try:
            img = Image.open(path)
            img = img.resize((thumb_width, thumb_height), Image.Resampling.LANCZOS)
            images.append(img)
        except PermissionError:
            print(f"Permission denied: {path}")
        except Exception as e:
            print(f"Error opening {path}: {e}")
    else:
        print(f"File not found: {path}")

print(f"Loaded {len(images)} images out of {len(image_paths)} found.")

rows = (len(images) + cols - 1) // cols

collage = Image.new('RGB', (cols*thumb_width, rows*thumb_height), color=(255,255,255))

for index, img in enumerate(images):
    x = (index % cols) * thumb_width
    y = (index // cols) * thumb_height
    collage.paste(img, (x, y))

if not images:
    raise ValueError("No valid images to create a collage!")

if cols*thumb_width == 0 or rows*thumb_height == 0:
    raise ValueError("Collage dimensions are zero! Check columns and thumbnail sizes.")


collage.save('collage.jpg')