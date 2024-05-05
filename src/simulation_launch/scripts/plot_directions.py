#!/usr/bin/python3

import datetime
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

        # for estimations, simulations in self.data:
        #     data.append([
        #         np.array([
        #             [
        #                 position.x, position.y,
        #                 # euler_from_quaternion(position.a, position.b, position.c, position.d)[2]
        #             ] for position in estimations.odometry_data
        #         ]),
        #         np.array([
        #             [
        #                 position.x, position.y,
        #                 # euler_from_quaternion(position.a, position.b, position.c, position.d)[2]
        #             ] for position in simulations.odometry_data
        #         ]),
        #     ])

        for step in self.data:
            estimation = np.array([[position.x, position.y] for position in step[0].odometry_data])
            simulation = np.array([[position.x, position.y] for position in step[1].odometry_data])

            data.append([estimation, simulation])

        return data


if __name__ == '__main__':
    # Seed 1 : 124
    # Seed 2 : 042
    # Seed 3 : 427
    # Seed 4 : 097
    # Seed 5 : 172

    # seeds = [124]
    seeds = [124, 42, 427, 97, 172]
    drops = [0.90]
    # drops = [0.00, 0.25, 0.50, 0.75, 0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    errors = [0.0]
    # errors = [0.00, 0.05, 0.10, 0.15]

    limit = (100, 300)  # 300 => 60 seconds
    flip_test = True
    file_directory = f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/output/directions"

    iterations = 20
    duration = 120
    start = 0

    mds = False
    pf = True

    subplot = True

    init = [False]
    offset = [False]
    certainty = [False]

    batch_plot = 1
    plot_grid = False  # To plot the last estimation of a given batch

    time = np.arange(0, limit[1]) / 5

    last_estimation = None
    last_simulation = None

    # f = open("output.csv", "w+")

    plt.figure(figsize=(10, 5))

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

            mean_square_error = []

            # Read the file
            for batch, seed in enumerate(seeds):
                file_name = f"drop_{drop:0.2f}_iteration_{iterations}_seed_{seed}_error_{error}_duration_{duration}_start_{start}"

                try:
                    file_reader = FileReader(f"{directory}/{file_name}")
                except Exception as e:
                    print(f"Couldn't read {file_name}", e)
                    continue

                mses = []
                flips = []

                print("[0] File :", file_reader.file_path)
                print("[0] File name :", file_reader.file_name)

                for est, sim in file_reader.get_positions_and_directions():
                    est = est[:, 0:2]
                    sim = sim[:, 0:2]

                    # TODO: add the rotation of the direction estimation to compare to the simulation
                    est, flip = rotate_and_translate(sim, est)
                    mse = np.mean(np.square(est - sim))

                    if batch == batch_plot:
                        last_estimation = est
                        last_simulation = sim

                    mses.append(mse)
                    flips.append(flip)

                # If MSEs is shorter than limit, add last value to fill up
                while len(mses) < limit[1]:
                    mses = mses + [mses[-1]]

                mses = mses[limit[0]:limit[1]]
                flips = flips[limit[0]:limit[1]]

                flips = np.array(flips)

                filtered_time = [time[limit[0]:limit[1]][i] for i in range(len(flips)) if flips[i]]
                filtered_mses = [mses[i] for i in range(len(flips)) if flips[i]]

                # f.write(f"{experiment.split('/')[1]},{drop},{error},{batch},{len(filtered_time)},{len(flips)}\n")

                if subplot:
                    plt.plot(time[limit[0]:limit[1]], mses, label=f"Batch {batch}: {len(filtered_time)}/{len(flips)}", alpha=0.5)
                    plt.scatter(filtered_time, filtered_mses, c="r", s=1)

                # plt.show()

                mean_square_error.append(mses)

            if not mean_square_error:  # or not flipped_mean_square_error:
                raise ValueError("Nothing to plot")

            # Average the result for each time_step
            mean_square_error = np.mean(np.array(mean_square_error), axis=0)

            # Plot the Mean Square Error
            if not plot_grid:
                label = ", ".join(experiment.split("/")[1].split("_")) + f", drop = {drop}, error = {error}"
                plt.plot(time[limit[0]:], mean_square_error, label=label)

    # f.close()

    # Plot a grid under the plot
    if plot_grid:
        plt.grid(True)

        plt.scatter(last_estimation[:, 0], last_estimation[:, 1], c="r", label="Estimation")
        for i, p in enumerate(last_estimation):
            plt.text(p[0] + 1, p[1] + 1, i, c="r")

        plt.scatter(last_simulation[:, 0], last_simulation[:, 1], c="b", label="Reality")
        for i, p in enumerate(last_simulation):
            plt.text(p[0] + 1, p[1] + 1, i, c="b")

        plt.title(f"Wrongly Estimated Positions (4 Moving Agents, {'MDS' if mds else 'PF'})")

        # Axis labels
        plt.xlabel("X-axis (cm)")
        plt.ylabel("Y-axis (cm)")

        # Equal ratio
        plt.axis('equal')

        plt.legend()
        plt.savefig(f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/mse_static_positions.png", dpi=300)
    else:
        # Title
        plt.title(f"MSE of Positions (4 Moving Agents, {'MDS' if mds else 'PF'})")

        # Axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Mean Square Error (cm²)")

        # Legend
        plt.legend()

        # Simply save with a timestamp
        plt.savefig(f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/scripts/plot/plot_mse_positions.png", dpi=300)

    plt.show()
