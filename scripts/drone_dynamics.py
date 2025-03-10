import numpy as np

def drone_dynamics(state, control, dt=0.05):
    """
    Simulates the dynamics of a drone.

    Parameters:
    state (numpy array): [px, py, pz, vx, vy, vz, yaw] - Drone's current state in global frame.
    control (numpy array): [ax, ay, az, yaw_rate] - Control inputs in local frame.
    dt (float): Time step for simulation.

    Returns:
    numpy array: Updated state after dt seconds.
    """
    # Unpack state and control
    px, py, pz, vx, vy, vz, yaw = state
    ax_global, ay_global, az_global, yaw_rate = control

    # Coordinate transformation for acceleration from local to global frame
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    # ax_global = ax_local * cos_yaw - ay_local * sin_yaw
    # ay_global = ax_local * sin_yaw + ay_local * cos_yaw
    # az_global = az_local

    # Update global velocity
    vx += ax_global * dt
    vy += ay_global * dt
    vz += az_global * dt

    # Update global position
    px += vx * dt
    py += vy * dt
    pz += vz * dt

    # Update yaw
    yaw += yaw_rate * dt

    # Wrap yaw to the range [-pi, pi]
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi

    # Return updated state
    return np.array([px, py, pz, vx, vy, vz, yaw])

# Example usage
if __name__ == "__main__":
    # Initial state: px, py, pz, vx, vy, vz, yaw
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Control inputs: ax, ay, az, yaw_rate
    control = np.array([1.0, 0.0, 0.0, 1])  # Move forward and turn slowly

    # Time step
    dt = 0.1

    # Simulate for 10 steps
    for _ in range(50):
        state = drone_dynamics(state, control, dt)
        print(f"State: {[round(i, 1) for i in state]}")
