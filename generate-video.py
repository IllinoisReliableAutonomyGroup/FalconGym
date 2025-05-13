import cv2
import os

# Specify the folder containing the images
image_folder = './visualize_trajectory_testing/'
output_video = 'sample.mp4'

# Get a list of all image files in the folder, sorted by name
images = sorted([img for img in os.listdir(image_folder) if img.endswith(".png")])

# Read the first image to get the size
first_image_path = os.path.join(image_folder, images[0])
frame = cv2.imread(first_image_path)
height, width, layers = frame.shape

# Define the codec and create the VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for mp4 files
video = cv2.VideoWriter(output_video, fourcc, 30, (width, height))  # 30 FPS

# Loop over all images and add them to the video
for image in images:
    img_path = os.path.join(image_folder, image)
    frame = cv2.imread(img_path)
    video.write(frame)

# Release the video writer
video.release()
print(f"Video saved as {output_video}")
