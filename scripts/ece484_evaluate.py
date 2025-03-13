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

def create_arrow(start, end, color=[0, 1, 0]):
    """Creates an arrow from start to end."""
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cone_radius=0.05, cone_height=0.1, cylinder_radius=0.02, cylinder_height=0.2
    )
    arrow.paint_uniform_color(color)
    
    direction = end - start
    direction /= np.linalg.norm(direction)  
    rotation = R.align_vectors([direction], [[0, 0, 1]])[0].as_matrix()  
    arrow.rotate(rotation, center=(0, 0, 0))
    arrow.translate(start)
    
    return arrow

def create_text_3d(text, position, font_size=50, color=[1, 1, 1]):
    """Creates a 3D text object."""
    text_3d = o3d.geometry.Text3D(text, position=position, font_size=font_size, color=color)
    return text_3d

def plot_with_matplotlib(trajectory, gates):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='blue', label="Trajectory")

    # Plot gates
    for gate_name, (gx, gy, gz, _, _, _) in gates.items():
        ax.scatter(gx, gy, gz, color='red', s=50, label=f"{gate_name}")
        ax.text(gx, gy, gz, gate_name, color='black', fontsize=12)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory and Gates")
    plt.legend()
    plt.show()

def create_text_marker(position, color=[1, 0, 1]):
    """Creates a small sphere to act as a text marker."""
    marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    marker.paint_uniform_color(color)
    marker.translate(position)
    return marker

def plot_trajectory_and_gates(trajectory, gates):
    objects = []

    # Create trajectory line
    trajectory_line = create_trajectory_line(trajectory)
    objects.append(trajectory_line)

    # Start and End Arrows
    start_arrow = create_arrow(trajectory[0, :3], trajectory[1, :3], color=[0, 1, 0])
    end_arrow = create_arrow(trajectory[-2, :3], trajectory[-1, :3], color=[0, 0, 1])
    objects.extend([start_arrow, end_arrow])

    # Add gates
    for gate_name, (gx, gy, gz, _, _, gyaw) in gates.items():
        gate_mesh = create_gate(gx, gy, gz, gyaw)
        objects.append(gate_mesh)

        # Add a small sphere above the gate as a placeholder for text
        marker = create_text_marker((gx, gy, gz + 0.5))
        objects.append(marker)

        print(f"Gate: {gate_name} at ({gx}, {gy}, {gz})")  # Print names in console

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
    timelist = []
    gatecrossed = []
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
                    timelist.append(i)
                    gatecrossed.append(gate2pass[j])
                elif distance_from_gate < gate_radius +1:
                    print(f"At trajectory point {i}, the drone missed the plane of gate {gate2pass[j]}.")


            prev_signs[j] = distances_to_planes[j]
    
    if gatecrossed[-1] != "Gate D":
        print("Race incomplete")
        return None
    elif gatecrossed[0] != "Gate A":
        print("Race did not begin from gate A ")
        return None

    avg_distance = np.mean(crossing_distances) if crossing_distances else None
    print(f"\nAverage distance from gate centers: {avg_distance:.2f} meters" if avg_distance else "No crossings detected.")
    print("Time taken : ",(timelist[-1] -timelist[0])*0.05, " seconds")
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
    #plot_with_matplotlib(trajectory, gates)


    plot_trajectory_and_gates(trajectory, gates)