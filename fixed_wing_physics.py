import numpy as np


def dynamics(state, action):
    """
    State  = [x, y, z, yaw, pitch, roll, vx, vy, vz]
    Action = [throttle u_T, aileron angle, yaw rate, pitch angle]
    """

    # -------------- parameters you will tune -----------------
    DT   = 1/30          # integration step  (s)
    G    = 9.81          # gravity           (m s‑2)

    # attitude & coupling
    K_ROLL        = 2.5
    K_YAW_COUPLE  = -0.5
    K_PITCH_TRACK = 6.0

    # speed & thrust
    K_THR_ACCEL = 25.0   # throttle → forward acceleration
    K_DRAG      = 0.08   # linear drag (crude)
    THR_TRIM    = 0.5    # throttle that just holds altitude straight & level

    # new climb / sink terms
    K_THR_CLIMB  = 1.0   # how much climb you get per unit excess throttle  (m s‑1)
    V_STALL      = 1.5  # m s‑1 – below this we start descending
    K_STALL_SINK = 4   # max sink rate at zero speed (m s‑1)
    # ----------------------------------------------------------

    # ---------------- unpack ---------------
    x, y, z, theta, gamma, phi, vx, vy, vz = state
    u_T, delta_a, u_theta_dot, gamma_cmd   = action

    # ----- 1. attitude rates -----
        # ---------- 1. roll dynamics with auto‑level ----------
    if abs(delta_a) < 0.1:
        # stick nearly centered → hold wings level
        phi_dot = 0.5 * (-phi)          # drives roll → 0
    else:
        # pilot actively commanding bank
        phi_dot = K_ROLL * delta_a              # nominal response
    v_mag     = max(np.linalg.norm([vx, vy, vz]), 1e-3)
    theta_dot = u_theta_dot + -(G / v_mag) * np.tan(phi) + K_YAW_COUPLE * delta_a
    gamma_dot = K_PITCH_TRACK * (gamma_cmd - gamma)

    # integrate Euler angles
    phi   += phi_dot   * DT
    theta += theta_dot * DT
    gamma += gamma_dot * DT

    # ----- 2. speed update -----
    v_dot  = K_THR_ACCEL * (u_T - THR_TRIM) - K_DRAG * v_mag
    v_new  = max(v_mag + v_dot * DT, 0.0)

    # ----- 3. velocity components -----
    vx = v_new * np.cos(gamma) * np.cos(theta)
    vy = v_new * np.cos(gamma) * np.sin(theta)
    # start with “kinematic” climb/descent from pitch
    vz = v_new * np.sin(gamma)

    # ----  NEW: throttle & stall effects on climb  ----
    # a) throttle coupling (positive if u_T > THR_TRIM)
    vz += K_THR_CLIMB * (min(u_T - THR_TRIM, 0))
    # b) sink if below stall
    if v_new < V_STALL:
        vz -= K_STALL_SINK * (V_STALL - v_new) / V_STALL
        # print("Stall", v_new, vz)

    # -----------------------------------------------

    # ----- 4. position update -----
    x += vx * DT
    y += vy * DT
    z += vz * DT

    return [x, y, z, theta, gamma, phi, vx, vy, vz]

def normalize_angle(angle):
    """
    Normalizes the angle to be within the range [-pi, pi].
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def landing_controller(state, target):

    # --- tuning gains (start conservative) ---
    K_pos_xy  = 0.2        # how hard we “bank toward” lateral error
    K_yaw     = 6         # closes yaw‑rate loop
    K_alt     = 0.2         # climb/descent throttle gain
    K_speed   = 0.1         # pitch‑angle to hold speed
    REF_SPEED = 18.0        # desired approach speed  (m s‑1)
    THR_IDLE  = 0.5
    THR_FULL  = 1.0
    THR_TRIM  = 0.5
    # -----------------------------------------


    x,y,z,theta,gamma,phi,vx,vy,vz = state
    target_yaw = target[-1]

    # ---------- 1) lateral guidance ----------
    
    pos     = np.array([x,y])
    tgt_xy  = target[:2]
    err_xy  = tgt_xy - pos                       # 2‑D vector to touchdown
    cross   = -np.cross([np.cos(theta), np.sin(theta),0],
                       [err_xy[0],err_xy[1], 0])[2] # signed

    # Compute heading and altitude errors
    goal_x, goal_y = target[:2]
    desired_theta = np.arctan((goal_y - y) / (goal_x - x))
    if (goal_y-y) < 0 and (goal_x-x) <0 :
        desired_theta = desired_theta - np.pi
    if (goal_y-y) > 0 and (goal_x-x) < 0 :
        desired_theta = desired_theta + np.pi

    heading_err = normalize_angle(desired_theta - theta)

    target_x = target[0]

    # # Use a stanley like for Aileron
    # if np.arctan((target[1] - y ) / (target[0] - x)) > 0:
    #     heading_err = normalize_angle(np.arctan((target[1] - y ) / (target[0] - x)) - theta)
    # else:
    #     heading_err = normalize_angle(np.arctan((target[1] - y ) / (target[0] - x)) + theta)
    #     # heading_err = normalize_angle(heading_err)
    CTE =  -heading_err - np.arctan(0.8*(x-target_x) / (0.1 + np.sqrt(vx**2+vy**2+vz**2)))
    print(heading_err, np.arctan((target[1] - y ) / (target[0] - x)), theta, CTE)

    # bank toward the runway centreline
    # delta_a = np.clip(K_pos_xy * cross, -1, 1)
    delta_a = np.clip(CTE, -1, 1)
    # print(f"CTE: {CTE}")
    u_theta_dot = 0
    # if np.abs(delta_a) < 0.2:
    #     # yaw‑rate command to align nose with runway direction (+y)
    yaw_err    = (target_yaw - theta + np.pi) % (2*np.pi) - np.pi
    u_theta_dot = np.clip(K_yaw * yaw_err, -1.5, 1.5)  # rad/s


    # ---------- 2) longitudinal guidance ----------
    # altitude error (positive if we are too high)
    alt_err  = z - target[2]
    # simple P controller on throttle around trim
    u_T = np.clip(THR_TRIM - K_alt * alt_err, THR_IDLE, THR_FULL)

    # hold approach speed with pitch
    speed    = np.linalg.norm([vx,vy,vz])
    y_cmd    = np.clip(K_speed * (speed - REF_SPEED), -0.15, 0.15)

    return (u_T, delta_a, u_theta_dot, y_cmd)

def testing_dynamics():
    steps = 60

    state = [-3, -20, 3, 1.57, 0, 0, 0, -8, 0]
    high_action = (0.3, 0.0, 0., 0.2)         # throttle well above trim
    # Throttle, aileron, rudder, pitch
    # aileron +: right turn
    # rudder +: left turn
    # pitch +: nose up

    with open("fixed_wing_traj.txt", 'w') as f:
        for _ in range(steps):
            state = dynamics(state, high_action)
            x, y, z, yaw, pitch, roll, vx, vy, vz = state
            f.write(f"{x} {y} {z} {yaw} {pitch} {roll} {vx} {vy} {vz}\n")

# ------------------------------------------------------------------
# quick demonstration: zero‑pitch level wings, throttle sweep
# ------------------------------------------------------------------
if __name__ == "__main__":
    # testing_dynamics()

    state = [2, -30, 0.5, 1.57, 0, 0, 0, 6, 0]

    # landing target in world coords
    target = np.array([-2.2, 0.0, -1.25, np.pi/2])   # x, y, z, yaw
    # target_yaw   = np.pi/2                  # want nose pointing +y
    # target_pitch = 0.0
    # target_roll  = 0.0

    with open("fixed_wing_traj.txt", 'w') as f:
        for idx in range(180):
            action = landing_controller(state, target)
            state = dynamics(state, action)
            print(idx, action, state[:3])
            x, y, z, yaw, pitch, roll, vx, vy, vz = state
            f.write(f"{x} {y} {z} {yaw} {pitch} {roll} {vx} {vy} {vz}\n")

            if z <= -1.25:
                break