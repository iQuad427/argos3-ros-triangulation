#!/usr/bin/python3

import datetime
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from utils import rotate_and_translate, euler_from_quaternion
from simulation_utils.msg import Positions, Odometry


def parse_positions(line: str) -> (Positions, bool):
    msg = Positions()
    msg.odometry_data = []

    faulty_frame = False

    infos = line.split("#")

    if len(infos) > 1:
        for info in infos:
            agent_id, positions, orientations = info.split(":")
            x, y, z = [float(i) for i in positions.split(",")]
            a, b, c, d = [float(j) for j in orientations.split(",")]

            msg.odometry_data.append(
                Odometry(
                    id=ord(agent_id),
                    x=x, y=y, z=z,
                    a=a, b=b, c=c, d=d
                )
            )
    else:
        faulty_frame = True

    return msg, faulty_frame


def unparse_positions(msg: Positions) -> str:
    line = f""
    for position in msg.odometry_data:
        if line:
            line += "#"
        line += f"{chr(position.id)}:{position.x},{position.y},{position.z}:{position.a},{position.b},{position.c},{position.d}"
    return f"{line}"


class FileReader:
    """
    Read a file like the following :
    type=relative_time=time&id,x,y#id,x,y#id,x,y#id,x,y ...

    And allow for doing some operations on it.
    """

    def __init__(self, file_path):
        self.file_path = file_path
        self.file_name = file_path.split("/")[-1]
        self._read_file(file_path)

    def _read_file(self, file_path):
        with open(file_path, "r") as f:
            lines = f.readlines()

        data = []
        time_values = []
        # By stem of 2 because we have 2 lines for each time
        for _j in range(0, len(lines), 2):
            _, timestamp, estimation = lines[_j].split("\n")[0].split("=")
            _, _, simulation = lines[_j + 1].split("\n")[0].split("=")

            time_values.append(float(timestamp))
            time_step = [
                parse_positions(estimation)[0],
                parse_positions(simulation)[0]
            ]

            data.append(time_step)

        self.data = data
        self.time = time_values

    def get_positions_and_directions(self):
        """
        :return: positions over time in numpy array format
        """
        data = []

        for estimations, simulations in self.data:
            data.append([
                estimations.timestamp,
                np.array([
                    [
                        position.x, position.y,
                        euler_from_quaternion(position.a, position.b, position.c, position.d)[2]
                    ] for position in estimations.odometry_data
                ]),
                np.array([
                    [
                        position.x, position.y,
                        euler_from_quaternion(position.a, position.b, position.c, position.d)[2]
                    ] for position in simulations.odometry_data
                ]),
            ])

        # for step in self.data:
        #     estimation = np.array([[position.x, position.y] for position in step[0].odometry_data])
        #     simulation = np.array([[position.x, position.y] for position in step[1].odometry_data])
        #
        #     data.append([estimation, simulation])

        return data


if __name__ == '__main__':
    # Seed 1 : 124
    # Seed 2 : 042
    # Seed 3 : 427
    # Seed 4 : 097
    # Seed 5 : 172

    seeds = [124]
    # seeds = [124, 42, 427, 97, 172]
    drops = [0.90]
    # drops = [0.00, 0.25, 0.50, 0.75, 0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    errors = [0.0]
    # errors = [0.00, 0.05, 0.10, 0.15]

    limit = (0, 600)  # 300 => 60 seconds for 10 iteration
    flip_test = True
    file_directory = f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/output/directions_distances"

    iterations = 20
    duration = 120
    start = 0

    mds = False
    pf = True

    subplot = False
    show_flip = False

    init = [False]
    offset = [False]
    certainty = [False]

    angular_speed = []
    linear_speed = []

    # plot = "positions"
    # plot = "mse_positions"
    plot = "mse_directions"

    batch_plot = 1
    plot_grid = False  # To plot the last estimation of a given batch

    time = np.arange(0, limit[1]) / 20

    last_estimation = None
    last_directions_estimation = None
    last_simulation = None
    last_directions_simulation = None

    # f = open("output.csv", "w+")

    plt.figure(figsize=(10, 6))

    # List of all combination of three False/True combinations
    method_experiments = []
    if mds:
        for i in init:
            for j in offset:
                for k in certainty:
                    output_dir = f"mds/mds"
                    output_dir += f"_init" if i else ""
                    output_dir += f"_offset" if j else ""
                    output_dir += f"_certainty" if k else ""

                    method_experiments.append(output_dir)

    if pf:
        for i in init:
            for j in offset:
                for k in certainty:
                    output_dir = f"pf_particles_5000_std_10_dt_0.1/pf"
                    output_dir += f"_init" if i else ""
                    output_dir += f"_offset" if j else ""
                    output_dir += f"_certainty" if k else ""

                    method_experiments.append(output_dir)

    config_experiments = []
    for drop in drops:
        for error in errors:
            config_experiments.append((drop, error))

    for experiment in method_experiments:
        directory = f"{file_directory}/{experiment}"

        # Check if the directory exists
        path = Path(directory)
        if not path.is_dir():
            print(f"{experiment} skipped")
            continue

        for drop, error in config_experiments:

            positions_error = []
            direction_error = []

            # Read the file
            for batch, seed in enumerate(seeds):
                file_name = f"drop_{drop:0.2f}_iteration_{iterations}_seed_{seed}_error_{error}_duration_{duration}_start_{start}"

                try:
                    file_reader = FileReader(f"{directory}/{file_name}")
                except Exception as e:
                    print(f"Couldn't read {file_name}", e)
                    continue

                print("[0] File :", file_reader.file_path)
                print("[0] File name :", file_reader.file_name)

                previous_time = None
                previous_ang = None
                previous_pos = None

                for timestamp, est, sim in file_reader.get_positions_and_directions():
                    angle_est = est[:, 2]
                    angle_sim = sim[:, 2]

                    directions_est = np.array([[np.cos(angle), np.sin(angle)] for angle in angle_est])
                    directions_sim = np.array([[np.cos(angle), np.sin(angle)] for angle in angle_sim])

                    positions_est = est[:, 0:2]
                    positions_sim = sim[:, 0:2]

                    positions_est, directions_est, flip = rotate_and_translate(
                        positions_sim, positions_est, directions=directions_est
                    )

                    # Come back to an angle from direction_est
                    direction = directions_est[0]
                    angle_estimated = math.atan2(
                        direction[1], direction[0]
                    )

                    direction_error = np.abs((angle_sim[0] - angle_estimated)) * 180/np.pi
                    direction_error = direction_error if direction_error < 180 else direction_error - 360

                    if previous_ang is not None:  # and previous_pos is not None:
                        speed = (angle_sim[0] - previous_ang[0]) / (time[1] - time[0])

                        if speed > 0:
                            speed = 20
                        elif speed < 0:
                            speed = -20

                        angular_speed.append(speed)
                        # linear_speed.append((previous_pos[0] - positions_sim[0]) / (time[1] - time[0]))

                    previous_ang = np.copy(angle_sim)
                    previous_pos = np.copy(positions_sim)

                    if batch == batch_plot:
                        last_estimation = positions_est
                        last_directions_estimation = directions_est
                        last_simulation = positions_sim
                        last_directions_simulation = directions_sim

    if plot == "mse_positions":
        # Title
        plt.title(f"MSE of Positions")

        # Axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Mean Square Error (cm²)")

        # Legend
        plt.legend()

        # Simply save with a timestamp
        plt.savefig(
            f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/plot_mse_positions.png",
            dpi=300
        )
    if not plot_grid and plot == "mse_directions":
        # Title
        plt.title(f"Error of Direction")

        # Add a redline at y = 0
        plt.axhline(y=0, color='r', linestyle='--', label="perfect estimation")

        # Plot angular speed
        plt.plot(angular_speed, color="tab:orange", label="angular speed")

        # Axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Error (°)")

        # Limit y-axis to -180/+180
        plt.ylim(-90, 90)

        # Legend
        plt.legend()

        # Simply save with a timestamp
        plt.savefig(
            f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/plot_mse_directions.png",
            dpi=300
        )

    plt.show()

