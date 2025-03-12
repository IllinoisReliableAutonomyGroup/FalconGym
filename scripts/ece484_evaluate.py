import numpy as np
import csv
import argparse
import json
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from scipy.spatial.transform import Rotation as R


GATE_RADIUS = 0.38 


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

    


def plot_trajectory_and_gates(trajectory, gates):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'b-', label="Trajectory")

    for gate_name, (gx, gy, gz, _, _, gyaw) in gates.items():

        ax.scatter(gx, gy, gz, color='r', s=50, label=f"{gate_name}") 
        
        theta = np.linspace(0, 2 * np.pi, 100)
        gate_x = gx + GATE_RADIUS * np.cos(theta)
        gate_y = gy + GATE_RADIUS * np.sin(theta)
        gate_z = np.full_like(gate_x, gz)  
        ax.plot(gate_x, gate_y, gate_z, 'r')
        arrow_length = 0.5
        ax.quiver(gx, gy, gz, arrow_length * np.cos(gyaw), arrow_length * np.sin(gyaw), 0, color='k', linewidth=1)


    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Trajectory and Gates Visualization")
    ax.legend()
    plt.show()


def create_trajectory_line(trajectory):
    """Creates a 3D line set for the trajectory."""
    points = trajectory[:, :3] 
    lines = [[i, i+1] for i in range(len(points)-1)]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1] for _ in lines]) 

    return line_set

def create_gate(x, y, z, theta):
    """Creates a 3D circular gate."""
    mesh = o3d.geometry.TriangleMesh.create_torus(torus_radius=GATE_RADIUS, tube_radius=0.05)
    mesh.paint_uniform_color([1, 0, 0]) 
    rotation_matrix = np.array([
    [0, 0, -1], 
    [0, 1, 0],
    [1, 0, 0]  
    ])

    mesh.rotate(rotation_matrix, center=(0, 0, 0))

        
    R = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0],
    [0, 0, 1]
    ])
    mesh.rotate(R, center=(0, 0, 0))
   
    mesh.translate((x, y, z))

    return mesh

def plot_trajectory_and_gates(trajectory, gates):
    """Visualizes the trajectory and gates in Open3D."""
    objects = []

    trajectory_line = create_trajectory_line(trajectory)
    objects.append(trajectory_line)


    for _, (gx, gy, gz, _, _, gyaw) in gates.items():
        gate_mesh = create_gate(gx, gy, gz, gyaw)
        objects.append(gate_mesh)


    o3d.visualization.draw_geometries(objects, window_name="3D Trajectory and Gates")




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
    for i in gate2pass:
        gx, gy, gz, pitch, roll, yaw = gates[i]
        gate_vectors.append((gx, gy, gz))
    
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
                elif distance_from_gate < gate_radius +1:
                    print(f"At trajectory point {i}, the drone missed the plane of gate {gate2pass[j]}.")


            prev_signs[j] = distances_to_planes[j]
  
    avg_distance = np.mean(crossing_distances) if crossing_distances else None
    print(f"\nAverage distance from gate centers: {avg_distance:.2f} meters" if avg_distance else "No crossings detected.")
    return avg_distance

def parse_args():
    parser = argparse.ArgumentParser(description="NeRF Renderer for Camera Poses")
    parser.add_argument("--track-name", type=str, required=True,
                        help="Specify track name [Circle_Track, Uturn_Track, Lemniscate_Track ] ")

    return parser.parse_args()




if __name__ == "__main__":
    args = parse_args()
    track = args.track_name

    filename = "circle_traj.txt"  # change this to ur text file to evaluate
    trajectory = read_trajectory(filename)
    gates = get_gate_coordinates(track)
    print(gates)
    evaluate_trajectory(trajectory, gates)


    plot_trajectory_and_gates(trajectory, gates)