import math
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib
import csv
import datetime

# Dodaj wybieranie jednostki czasu


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


    filenames = ['pid', 'pure_pursuit']

    for filename in filenames:
        print(filename)
        data = np.genfromtxt(f'../../pos_logs/{filename}.txt', delimiter=',', skip_header=1)

        time = data[:, 0]
        Y = -data[:, 1] + 30.0
        X = data[:, 2] + 2.5


        n_points = min(len(time), 10_000)

        X = X[:n_points]
        Y = Y[:n_points]

        sum_error = 0

        for px, py in zip(X, Y):
            curr_point = np.array([px, py])
            distances = np.linalg.norm(track - curr_point, axis=1)
            error = np.min(distances)
            sum_error += error
        
        prev_points = track[:-1]
        next_points = track[1:]
        whole_distance = np.linalg.norm(next_points - prev_points)

        relative_err = sum_error / whole_distance

        now = datetime.datetime.now()
        timestamp_str = now.strftime("%Y%m%d_%H%M%S")
        #filename = f"pos_graph_{timestamp_str}.png"

        plt.figure(figsize=(6, 6))
        plt.plot(track[:, 0], track[:, 1], marker='x', color='b', linestyle='-', label="Racetrack")
        plt.plot(X, Y, color='r', linewidth=1, linestyle='-',  label="Car Path")
        plt.title(f"Err: {sum_error}, rel_err:{relative_err}")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.savefig(f"plot_err_{filename}.png")

        print(f"Error: {sum_error}")
        print(f"Relative error: {relative_err}")


if __name__ == '__main__':
    main()
