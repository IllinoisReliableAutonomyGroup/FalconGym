import numpy as np
import cv2

def gate_detection(image):

    masks = np.zeros(image.shape)
    return masks


if __name__ == "__main__":
    image = cv2.imread('gate-detect-Yan-example/0050.png')

    mask = gate_detection(image)
    

    cv2.imshow('Original Image', image)
    cv2.imshow('mask', mask)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
