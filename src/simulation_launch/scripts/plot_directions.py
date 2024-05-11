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

        return data


if __name__ == '__main__':
    # Seed 1 : 124
    # Seed 2 : 042
    # Seed 3 : 427
    # Seed 4 : 097
    # Seed 5 : 172

    seeds = [42]
    # seeds = [124, 42, 427, 97, 172]
    drops = [0.50]
    # drops = [0.00, 0.25, 0.50, 0.75, 0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    errors = [0.15]
    # errors = [0.00, 0.05, 0.10, 0.15]

    plt.figure(figsize=(6, 4))
    limit = (0, 1200)  # 300 => 60 seconds for 10 iteration
    flip_test = True
    file_directory = f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/output/final_slow"

    iterations = 100
    duration = 120
    start = 0

    mds = True
    pf = True

    # plot = "positions"
    # plot = "mse_positions"
    plot = "mse_directions"

    subplot = False
    show_flip = False
    show_speed = True

    init = [True]
    # init = [False, True]
    offset = [False]
    # init = [False, True]
    certainty = [False]
    # certainty = [False, True]

    batch_plot = 1
    plot_grid = False  # To plot the last estimation of a given batch

    time = np.arange(0, limit[1]) / 20

    last_estimation = None
    last_directions_estimation = None
    last_simulation = None
    last_directions_simulation = None

    # f = open("output.csv", "w+")

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
        for i in [False]:
            for j in [False]:
                for k in [False]:
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

            mean_square_error = []
            mean_square_error_direction = []

            # Read the file
            for batch, seed in enumerate(seeds):
                file_name = f"drop_{drop:0.2f}_iteration_{iterations}_seed_{seed}_error_{error}_duration_{duration}_start_{start}"

                try:
                    file_reader = FileReader(f"{directory}/{file_name}")
                except Exception as e:
                    print(f"Couldn't read {file_name}", e)
                    continue

                mses = []
                mses_direction = []
                flips = []

                angular_speed = []

                print("[0] File :", file_reader.file_path)
                print("[0] File name :", file_reader.file_name)

                previous_ang = None
                # previous_pos = None

                for (t, est, sim) in file_reader.get_positions_and_directions():
                    angle_est = est[:, 2]
                    angle_sim = sim[:, 2]

                    directions_est = np.array([[np.cos(angle), np.sin(angle)] for angle in angle_est])
                    directions_sim = np.array([[np.cos(angle), np.sin(angle)] for angle in angle_sim])

                    positions_est = est[:, 0:2]
                    positions_sim = sim[:, 0:2]

                    positions_est, directions_est, flip = rotate_and_translate(
                        positions_sim, positions_est, directions=directions_est
                    )

                    mse = np.mean(np.square(positions_est - positions_sim))

                    # Come back to an angle from direction_est
                    direction = directions_est[0]
                    angle_estimated = math.atan2(
                        direction[1], direction[0]
                    )

                    mse_direction = np.abs(angle_sim[0] * 180/np.pi % 360 - angle_estimated * 180/np.pi % 360)
                    mse_direction = mse_direction if mse_direction < 180 else mse_direction - 360

                    # Take the absolute value
                    # mse_direction = np.abs(mse_direction)

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

                    mses.append(mse)
                    mses_direction.append(mse_direction)
                    flips.append(flip)

                # If MSEs is shorter than limit, add last value to fill up
                while len(mses) < limit[1]:
                    mses = mses + [mses[-1]]

                while len(mses_direction) < limit[1]:
                    mses_direction = mses_direction + [mses_direction[-1]]

                # while len(angular_speed) < limit[1]:
                #     angular_speed = angular_speed + [angular_speed[-1]]

                mses = mses[limit[0]:limit[1]]
                mses_direction = mses_direction[limit[0]:limit[1]]

                flips = flips[limit[0]:limit[1]]
                flips = np.array(flips)

                angular_speed = angular_speed[limit[0]:limit[1]]

                filtered_time = [time[limit[0]:limit[1]][i] for i in range(len(flips)) if flips[i]]
                filtered_mses = [mses[i] for i in range(len(flips)) if flips[i]]
                filtered_mses_direction = [mses_direction[i] for i in range(len(flips)) if flips[i]]

                # f.write(f"{experiment.split('/')[1]},{drop},{error},{batch},{len(filtered_time)},{len(flips)}\n")
                if show_speed:
                    plt.plot(time[limit[0]:limit[1]], angular_speed, label="agent rotating")

                if subplot and plot == "mse_positions":
                    plt.plot(time[limit[0]:limit[1]], mses, label=f"Batch {batch}: {len(filtered_time)}/{len(flips)}", alpha=0.5)
                    if show_flip:
                        plt.scatter(filtered_time, filtered_mses, c="r", s=1)
                if subplot and plot == "mse_directions":
                    plt.plot(time[limit[0]:limit[1]], mses_direction, label=f"Batch {batch}: {len(filtered_time)}/{len(flips)}", alpha=0.5)
                    if show_flip:
                        plt.scatter(filtered_time, filtered_mses_direction, c="r", s=1, label="flip detected")

                # plt.show()

                mean_square_error.append(mses)
                mean_square_error_direction.append(mses_direction)

            if not mean_square_error:  # or not flipped_mean_square_error:
                print("Nothing to plot")

            # Average the result for each time_step
            mean_square_error = np.mean(np.array(mean_square_error), axis=0)
            mean_square_error_direction = np.mean(np.array(mean_square_error_direction), axis=0)

            # Plot the Mean Square Error
            if plot == "mse_positions" and not plot_grid:
                label = ", ".join(experiment.split("/")[1].split("_")) + f", drop = {drop}, error = {error}"
                plt.plot(
                    time[limit[0]:],
                    mean_square_error,
                    label=label
                )
            elif plot == "mse_directions" and not plot_grid:
                label = ", ".join(experiment.split("/")[1].split("_")) + f", drop = {drop}, error = {error}"
                plt.plot(
                    time[limit[0]:],
                    mean_square_error_direction,
                    label=label
                )
                # if show_flip:
                #     plt.scatter(filtered_time, filtered_mses_direction, c="r", s=1, zorder=10, label="flip detected")

    # f.close()

    # Plot a grid under the plot
    if plot_grid and plot == "positions":
        plt.grid(True)

        arrow_size = 20

        plt.scatter(last_estimation[:, 0], last_estimation[:, 1], c="r", label="Estimation")
        for i, p in enumerate(last_estimation):
            plt.text(p[0] + 1, p[1] + 1, i, c="r")
            plt.arrow(p[0], p[1], arrow_size * last_directions_estimation[i, 0], arrow_size * last_directions_estimation[i, 1], head_width=5, head_length=5, fc='r', ec='r')

        plt.scatter(last_simulation[:, 0], last_simulation[:, 1], c="b", label="Reality")
        for i, p in enumerate(last_simulation):
            plt.text(p[0] + 1, p[1] + 1, i, c="b")
            plt.arrow(p[0], p[1], arrow_size * last_directions_simulation[i, 0], arrow_size * last_directions_simulation[i, 1], head_width=5, head_length=5, fc='b', ec='b')

        plt.title(f"Wrongly Estimated Positions (4 Moving Agents, {'MDS' if mds else 'PF'})")

        # Axis labels
        plt.xlabel("X-axis (cm)")
        plt.ylabel("Y-axis (cm)")

        # Equal ratio
        plt.axis('equal')

        plt.legend()
        plt.savefig(f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/mse_static_positions.png", dpi=300)
    if not plot_grid and plot == "mse_positions":
        # Title
        # plt.title(f"MSE of Positions (4 Moving Agents, {'MDS' if mds else 'PF'})")
        plt.title(f"Comparison of MDS and PF (err = {errors[0]})")

        # Axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Mean Square Error (cm²)")

        # Legend
        plt.legend()

        # Simply save with a timestamp
        plt.savefig(f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/plot_mse_positions.png", dpi=300)
    if not plot_grid and plot == "mse_directions":
        # Title
        plt.title(f"Error of Direction (Displacement Estimation)")
        # plt.title(f"Error of Direction (4 Moving Agents, {'MDS' if mds else 'PF'})")

        # Add a redline at y = 0
        plt.axhline(y=180, color='gray', linestyle='--')


        # Axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Error (°)")

        # Limit y-axis to -180/+180
        # plt.ylim(0, 250)

        # Legend
        plt.legend()

        # Simply save with a timestamp
        plt.savefig(f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/plot_mse_directions.png", dpi=300)

    plt.show()

    # # Plot the distribution of the mean error on the direction
    # plt.figure(figsize=(5, 5))
    #
    # # Remove unknown values
    # mean_square_error_direction = [mse for mse in mean_square_error_direction if not np.isnan(mse)]
    # # print(mean_square_error_direction)
    #
    # # Plot a gaussian kernel density estimate
    # mean, std = np.mean(mean_square_error_direction), np.std(mean_square_error_direction)
    # x = np.linspace(-180, 180, 1000)
    # y = (1 / (std * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mean) / std) ** 2)
    # plt.plot(x, y, color="r", label=f"Gaussian KDE, {mean:.2f}° ± {std:.2f}°")
    #
    # # Plot the histogram with another scale
    # plt.hist(mean_square_error_direction, bins=100, alpha=0.5, label="Mean Error Distribution", density=True)
    #
    # plt.title(f"Mean Error of Direction (Distances Estimation, {'MDS' if mds else 'PF'}))")
    # # plt.title(f"Mean Error of Direction (4 Moving Agents, {'MDS' if mds else 'PF'})")
    # plt.xlabel("Mean Error (°)")
    # plt.ylabel("Frequency")
    #
    # plt.legend()
    # plt.savefig(f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/hist_mse_directions.png", dpi=300)
    # plt.show()

