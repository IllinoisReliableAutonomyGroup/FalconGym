"""
File: ece484_videogenerator.py

Description:
    This script reads a sequence of images from a specified directory and creates a video.
    The images must be named sequentially (e.g., 1.png, 2.png, ...). The output video is
    saved in MP4 format.

Usage:
    python script.py --input <input_directory> --output <output_video_path> --fps <frames_per_second>

Arguments:
    --input   : Path to the directory containing PNG images.
    --output  : Path where the output video file will be saved.
    --fps     : Frames per second (default: 20).


"""

import os
import cv2
import argparse

def create_video_from_images(input_path, output_path, fps=20):
    """
    Creates a video from a sequence of images in the specified directory.
    
    Args:
        input_path (str): Path to the directory containing image files.
        output_path (str): Path where the output video file will be saved.
        fps (int, optional): Frames per second for the video. Default is 30.
    """
    # Get list of image files with .png extension
    image_files = [f for f in os.listdir(input_path) if f.endswith('.png')]
    
    if not image_files:
        print("No PNG images found in the specified directory.")
        return
    
    # Sort images by filename, assuming they are named sequentially
    image_files.sort(key=lambda x: int(x.split('.')[0]))
    
    # Read the first image to get dimensions
    first_image_path = os.path.join(input_path, image_files[0])
    img = cv2.imread(first_image_path)
    height, width, _ = img.shape
    
    # Define the video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # Write each image to the video
    for file in image_files:
        img_path = os.path.join(input_path, file)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Unable to read {img_path}")
            continue
        video_writer.write(img)
    
    # Release the video writer
    video_writer.release()
    print(f"Video created successfully: {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a video from images in a directory.")
    parser.add_argument("--input", required=True, help="Path to the input directory containing images.")
    parser.add_argument("--output", required=True, help="Path to save the output video file.")
    parser.add_argument("--fps", type=int, default=20, help="Frames per second (default: 20)")
    
    args = parser.parse_args()
    create_video_from_images(args.input, args.output, args.fps)