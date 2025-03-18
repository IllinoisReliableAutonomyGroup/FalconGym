import numpy as np
import csv
import argparse
import json
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as R
import plotly.graph_objects as go

GATE_RADIUS = 0.38 #gate Radius

def generate_gate_circle(gx, gy, gz, yaw, num_points=100):
    """Generates a circular gate in the XZ plane and rotates it by yaw, matching Open3D transformation."""
    theta = np.linspace(0, 2 * np.pi, num_points)  # 100 points around the circle
    z = GATE_RADIUS * np.cos(theta)  
    x = GATE_RADIUS * np.sin(theta)  

    x_rot = x  
    y_rot = np.zeros_like(x)  
    z_rot = -z  

    cos_yaw, sin_yaw = np.cos(yaw+3.14/2), np.sin(yaw+3.14/2)
    x_final = gx + (x_rot * cos_yaw - y_rot * sin_yaw)
    y_final = gy + (x_rot * sin_yaw + y_rot * cos_yaw)
    z_final = gz + z_rot  

    return x_final, y_final, z_final

def plot_with_plotly(trajectory, gates):
    fig = go.Figure()

    # Plot trajectory
    fig.add_trace(go.Scatter3d(
        x=trajectory[:, 0], 
        y=trajectory[:, 1], 
        z=trajectory[:, 2], 
        mode='lines', 
        line=dict(color='blue', width=6),
        name='Trajectory'
    ))

    for gate_name, (gx, gy, gz, _, _, gyaw) in gates.items():
        gate_x, gate_y, gate_z = generate_gate_circle(gx, gy, gz, gyaw)

        # Plot gate circl
        fig.add_trace(go.Scatter3d(
            x=gate_x, 
            y=gate_y, 
            z=gate_z, 
            mode='lines',
            line=dict(color='red', width=5),
            name=f'{gate_name} (Gate)'
        ))

        fig.add_trace(go.Scatter3d(
            x=[gx], 
            y=[gy], 
            z=[gz + GATE_RADIUS + 0.2], 
            mode='text',
            text=[gate_name],  
            textposition="middle center",
            showlegend=False
        ))


    fig.add_trace(go.Scatter3d(
        x=[trajectory[0, 0]], 
        y=[trajectory[0, 1]], 
        z=[trajectory[0, 2]], 
        mode='markers+text',
        marker=dict(color='green', size=10, symbol='diamond'),
        text=['Start'],
        textposition="bottom center",
        name='Start Point'
    ))

    fig.add_trace(go.Scatter3d(
        x=[trajectory[-1, 0]], 
        y=[trajectory[-1, 1]], 
        z=[trajectory[-1, 2]], 
        mode='markers+text',
        marker=dict(color='red', size=10, symbol='diamond'),
        text=['End'],
        textposition="top center",
        name='End Point'
    ))
    

    fig.update_layout(
        title="Trajectory and Gates",
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z",
            zaxis=dict(range=[-2, 2]),
        ),
        margin=dict(l=0, r=0, b=0, t=40)
    )

    fig.show()


def read_trajectory(filename):
    
    data = []
    with open(filename, 'r') as file:
        for line in file:
            values = list(map(float, line.split()))  
            data.append(values)

    return np.array(data)  


def get_gate_coordinates(track_name):
    filename = "./gates_poses.json"

    with open(filename, 'r') as file:
        gate_data = json.load(file)
    
    if track_name in gate_data:
        return gate_data[track_name]
    else:
        raise ValueError(f"Track name '{track_name}' not found in JSON file.")

    



def evaluate_trajectory(trajectory, gates, gate_radius=0.38, distance_threshold=0.5):


    gate_normal_list = []
    gate2pass = ["Gate A","Gate B","Gate C","Gate D"]
    for gate_name, (gx, gy, gz, pitch, roll, yaw) in gates.items():
        if yaw ==0:
            yaw =3.14
        rotation = R.from_euler('xyz', [pitch, roll, yaw])
        gate_normal = rotation.apply([yaw, 0, 0])  
        gate_normal_list.append(gate_normal)

    crossing_distances = []
    prev_signs = [None, None, None, None]  


    gate_vectors = []
    timelist = []
    gatecrossed = []
    for i in gate2pass:
        gx, gy, gz, pitch, roll, yaw = gates[i]
        gate_vectors.append((gx, gy, gz))
    sr = 0
    for i, point in enumerate(trajectory):
        px, py, pz = point[:3] 
        vectors_to_gates = [
            np.array([px - gx, py - gy, pz - gz]) for gx, gy, gz in gate_vectors
        ]


        distances_to_planes = [
            np.dot(vectors_to_gates[j], gate_normal_list[j]) for j in range(len(gate2pass))
        ]


        for j in range(len(gate2pass)):

            if prev_signs[j] is not None and np.sign(prev_signs[j]) != np.sign(distances_to_planes[j]):
                gx, gy, gz = gate_vectors[j] 
                distance_from_gate = np.linalg.norm([px - gx, py - gy, pz - gz])

                if distance_from_gate < gate_radius :
                    print(f"At trajectory point {i}, the drone crossed the plane of gate {gate2pass[j]}.")
                    print(f"Distance from gate center at crossing: {distance_from_gate:.2f} meters")
                    crossing_distances.append(distance_from_gate)
                    timelist.append(i)
                    gatecrossed.append(gate2pass[j])
                    sr = sr +1
                elif distance_from_gate < gate_radius +1:
                    print(f"At trajectory point {i}, the drone missed the plane of gate {gate2pass[j]}.")


            prev_signs[j] = distances_to_planes[j]
    
    if gatecrossed[-1] != "Gate D":
        print("Race incomplete")
        return None
    elif gatecrossed[0] != "Gate A":
        print("WARNING! Race did not begin from gate A ")
        #return None

    sr =sr/8
    lt = (timelist[-1] -timelist[0])*0.05
    avg_distance = np.mean(crossing_distances) if crossing_distances else None
    print(f"\nAverage distance from gate centers: {avg_distance:.2f} meters" if avg_distance else "No crossings detected.")
    print("Time taken : ",lt, " seconds")
    print("Success Rate ", sr)
    return avg_distance, lt, sr

def parse_args():
    parser = argparse.ArgumentParser(description="NeRF Renderer for Camera Poses")

    parser.add_argument("--track-name", type=str, required=True,
                        help="Specify track name [Circle_Track, Uturn_Track, Lemniscate_Track ] ")
    parser.add_argument("--trajectory-path", type=str, required=True,
                        help="txt file location of trajectory points saved")
    parser.add_argument("--visflag", type=bool, default=True, nargs='?', const=False,
                        help="Flag for visualization (default: False)")
    parser.add_argument("--metricsflag", type=bool, default=True, nargs='?', const=False,
                        help="Flag for metrics (default: False)")
    
    return parser.parse_args()




if __name__ == "__main__":
    args = parse_args()
    track = args.track_name
    filename = args.trajectory_path
    vflag = args.visflag
    mflag = args.metricsflag

    trajectory = read_trajectory(filename)
    gates = get_gate_coordinates(track)
    print(gates)
    avg_distance, lt, sr = evaluate_trajectory(trajectory, gates)

    if vflag:
        plot_with_plotly(trajectory, gates)


    if mflag:
        metrics = {
        "MSE (distance_error_from_gate_centers)": avg_distance,
        "Lap_time": lt,
        "Success_rate": sr
        }

        file_path = track+ '_metrics.json'  # You can change the path if needed
        with open(file_path, 'w') as json_file:
            json.dump(metrics, json_file, indent=4)

        print(f"Metrics saved to {file_path}")


    #plot_trajectory_and_gates(trajectory, gates)