import numpy as np

def vision_controller(image):
    control = [0, 0, 0 , 0] # ax, ay, az, yaw_rate
    return control

if __name__ == "__main__":
    image = np.zeros((480, 640, 3)) # PlaceHolder
    control = vision_controller(image)
    print(control)