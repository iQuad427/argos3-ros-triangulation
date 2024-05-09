import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from utils import rotate_and_translate, FileReader

if __name__ == '__main__':
    # Seed 1 : 124
    # Seed 2 : 042
    # Seed 3 : 427
    # Seed 4 : 097
    # Seed 5 : 172

    seeds = [124]
    # seeds = [124, 42, 427, 97, 172]
    drops = [0.50]
    # drops = [0.00, 0.25, 0.50, 0.75, 0.90, 0.95, 0.96, 0.97, 0.98, 0.99]
    errors = [0.0]
    # errors = [0.00, 0.05, 0.10, 0.15]

    limit = (0, 300)  # 300 => 60 seconds
    time = np.arange(0, limit[1]) / 5

    file_directory = f"/home/quentin/Dev/argos3-ros-triangulation/src/simulation_launch/output/final"
    iterations = 100

    mds = True
    pf = True

    subplot = False

    init = [False]
    offset = [False]
    certainty = [False]

    batch_plot = 0
    plot_grid = False  # To plot the last estimation of a given batch

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

    # method_experiments = [
    #     "static_convergence_mds",
    # ]

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
                file_name = f"drop_{drop:0.2f}_seed_{seed}_error_{error}_duration_{150}_start_{30}"

                try:
                    file_reader = FileReader(f"{directory}/{file_name}")
                except Exception as e:
                    print(f"Couldn't read {file_name}", e)
                    continue

                mses = []
                flips = []

                print("[0] File :", file_reader.file_path)
                print("[0] File name :", file_reader.file_name)

                for est, sim in file_reader.make_numpy():
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

        plt.title(f"Wrongly Estimated Positions (4 Static Agents, MDS)")

        # Axis labels
        plt.xlabel("X-axis (cm)")
        plt.ylabel("Y-axis (cm)")

        # Equal ratio
        plt.axis('equal')

        plt.legend()
        plt.savefig(f"./plot/mse_static_positions" + ".png", dpi=300)
    else:
        # Title
        plt.title(f"MSE of Positions")

        # Axis labels
        plt.xlabel("Time (s)")
        plt.ylabel("Mean Square Error (cm²)")

        # Legend
        plt.legend()

        # Simply save with a timestamp
        plt.savefig(f"../output/plot/plot_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}" + ".png", dpi=300)

    plt.show()
