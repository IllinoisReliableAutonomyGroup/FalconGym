
import numpy as np

from fixed_wing_physics import landing_controller, dynamics, normalize_angle

# state = [-2.2, -25, 0.8, 1.57, 0, 0, 0, 7, 0]

# state = [-1.5, -25, 0.5, 1.7, 0.2, 0.4, 0, 10, 0]
# state = [-2.2, -26, 0.2, 1.57, 0, 0, 0, 10, 0]

# state = [-2.2, -28, 0.1, 1.3, 0, 0, 0, 12, 0]
# state = [-2.2, -22, -0.4, 1.57, 0.4, 0, 0, 12, 0]

# state = [-2.3, -29, 0.3, 1.57, -0.1, 0.3, 0, 12, 0]
# state = [-1.7, -30, 0.4, 1.6, 0, 0.3, 0, 12, 0]

# state = [-2.5, -32, 0, 1.57, 0., 0, 0, 12, 0]
# state = [-1.9, -25, -0.2, 1.57, 0.4, 0, 0, 12, 0]

# state = [-2.3, -32, 0.5, 1.57, 0., 0, 0, 12, 0]
# state = [-2, -26, -0.8, 1.75, 0., 0, 0, 12, 0]


state = [1, -20, 2, 1.57, 0, 0, 0, 8, 0]
state = [4, -20, 2, 1.57, 0, 0, 0, 8, 0]
# state = [2.5, -20, 2, 1.57, 0, 0, 0, 8, 0]



# landing target in world coords
target = np.array([2.5, 0, -0.5, np.pi/2])   # x, y, z, yaw
# target_yaw   = np.pi/2                  # want nose pointing +y
# target_pitch = 0.0
# target_roll  = 0.0

with open("fixed_wing_traj.txt", 'w') as f:
    for idx in range(90):
        action = landing_controller(state, target)
        state = dynamics(state, action)
        print(idx, action, state[:3])
        x, y, z, yaw, pitch, roll, vx, vy, vz = state
        f.write(f"{x} {y} {z} {yaw} {pitch} {roll} {vx} {vy} {vz}\n")

        if z <= -0.8:
            break

# print(np.abs(x+2.2)/2)