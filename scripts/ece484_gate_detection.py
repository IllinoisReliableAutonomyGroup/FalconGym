"""
Gate Detection Script

This script reads an image, applies gate detection algorithmn,
and displays the original image along with the generated mask.

"""
import numpy as np
import cv2

def gate_detection(image):
    #TODO TASK 2 Write gate detection algorithmn here
    masks = np.zeros(image.shape)
    return masks


if __name__ == "__main__":
    #Use sample image fron ns-renderer.py to check you gate detection algorithmn.
    image = cv2.imread('gate-detect-Yan-example/0050.png')
    mask = gate_detection(image)
    

    cv2.imshow('Original Image', image)
    cv2.imshow('mask', mask)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
