import math
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib
import csv


# matplotlib.use('TkAgg')

def linear_interpolate(p1, p2, num_points=10):
    x_values = np.linspace(p1[0], p2[0], num_points)
    y_values = np.linspace(p1[1], p2[1], num_points)
    return x_values, y_values


def main():
    waypoints = np.load('center_line.npy', allow_pickle=True)

    track = waypoints[:, 0:2] * 12

    n_points = track.shape[1]

    pid_track_x = np.random.rand(n_points) * 60
    pid_track_y = np.random.rand(n_points) * 50


    data = np.genfromtxt('data.csv', delimiter=',', skip_header=1)

    time = data[:, 0]
    X = data[:, 1]
    Y = data[:, 2]

    sum_error = 0

    for px, py in zip(X, Y):
        curr_point = np.array([px, py])
        distances = np.linalg.norm(track - curr_point, axis=1)
        error = np.min(distances)
        sum_error += error

    plt.figure(figsize=(6, 6))
    plt.plot(track[:, 0], track[:, 1], marker='x', color='b', linestyle='-', label="LineString")
    plt.plot(pid_track_x, pid_track_y, marker='o', color='r', linestyle='-', label="LineString")
    plt.title("LineString from Shapely")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')  # To keep aspect ratio equal
    plt.show()

    print(f"Error: {sum_error}")


if __name__ == '__main__':
    main()
