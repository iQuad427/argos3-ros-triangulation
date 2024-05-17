import copy
import datetime

import numpy as np
import rosbag
from matplotlib import animation

from utils import euler_from_quaternion

if __name__ == '__main__':
    # input_bag = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_launch/output/exp_12-landmarked-batch-2/trilateration/trilateration_offset/seed_124_direction_displacement_historic_5.bag"
    input_bag = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_launch/output/exp_9-three-moving-batch-2/mds/mds_offset/seed_124_direction_displacement_historic_5.bag"
    saved_positions = None

    iteration_rate = 20

    topics = [
        "/compute/positions",
        "/experiment/positions"
    ]

    dict_data = {
        topic: [] for topic in topics
    }

    timestamps = []
    real_positions = []
    estimated_positions = []

    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic in topics:
            dict_data[topic].append(msg)

            if all([len(dict_data[top]) > 0 for top in topics]):
                # Save the positions to be able to plot it later on
                real = dict_data["/experiment/positions"][-1]
                estimated = dict_data["/compute/positions"][-1]

                # Save in lists
                timestamps.append(t)
                real_positions.append(real)
                estimated_positions.append(estimated)

                # Clear the dictionary
                dict_positions = {
                    f"{top}": [] for top in topics
                }

    # Animate side by side the plot of positions estimates and the real positions
    # Save the animation in a gif
    import matplotlib.pyplot as plt

    # Timestamp limited to duration of 120 seconds
    timestamps_new = []
    for timestamp in timestamps:
        if (timestamp - timestamps[0]).secs > 130:
            continue
        timestamps_new.append(timestamp)

    timestamps = copy.deepcopy(timestamps_new)

    fig, ax = plt.subplots(1, 2, figsize=(10, 5))

    # Top title
    plt.suptitle(f"Timestamp: 0s")

    ax[0].set_title("Real positions")
    ax[1].set_title("Estimated positions")

    ax[0].set(xlim=[-3, 3], ylim=[-3, 3], xlabel='X [m]', ylabel='Y [m]')
    ax[1].set(xlim=[-3, 3], ylim=[-3, 3], xlabel='X [m]', ylabel='Y [m]')

    # real_plot = ax[0].scatter(
    #     *np.stack(
    #         [(p.pose.position.x, p.pose.position.y) for p in real_positions[0].positions]
    #     ).T,
    # )
    #
    # estimated_plot = ax[1].scatter(
    #     *np.stack(
    #         [(p.pose.position.x / 100, p.pose.position.y / 100) for p in estimated_positions[0].positions]
    #     ).T
    # )

    def update(frame):
        ax[0].clear()
        ax[1].clear()

        ax[0].set(xlim=[-3, 3], ylim=[-3, 3], xlabel='X [m]', ylabel='Y [m]')
        ax[1].set(xlim=[-3, 3], ylim=[-3, 3], xlabel='X [m]', ylabel='Y [m]')

        frame = frame * iteration_rate

        timestamp = timestamps[frame]
        real_position = real_positions[frame]
        estimated_position = estimated_positions[frame]

        duration = (timestamp - timestamps[0])

        if duration.secs > 130:
            return

        plt.suptitle(f"Timestamp: {int(duration.secs) + (int(duration.nsecs) / 10e8):0.2f}s")

        positions = np.stack(
            [(p.pose.position.x, p.pose.position.y) for p in real_position.positions]
        )
        ax[0].scatter(positions[:, 0], positions[:, 1])

        # Plot orientations as vectors from the position
        for position in real_position.positions:
            coordinates = [position.pose.position.x, position.pose.position.y]
            orientation = euler_from_quaternion(
                position.pose.orientation.x,
                position.pose.orientation.y,
                position.pose.orientation.z,
                position.pose.orientation.w,
            )[2]

            direction = [np.cos(orientation), np.sin(orientation)]

            ax[0].quiver(*coordinates, *(np.array(coordinates) + np.array(direction)), color="green")

        positions = np.stack(
            [(p.pose.position.x / 100, p.pose.position.y / 100) for p in estimated_position.positions]
        )
        ax[1].scatter(positions[:, 0], positions[:, 1])

        for position in estimated_position.positions:
            coordinates = [position.pose.position.x / 100, position.pose.position.y / 100]
            orientation = euler_from_quaternion(
                position.pose.orientation.x,
                position.pose.orientation.y,
                position.pose.orientation.z,
                position.pose.orientation.w,
            )[2]

            print(orientation)

            direction = [np.cos(orientation), np.sin(orientation)]

            ax[1].quiver(*coordinates, *(np.array(coordinates) + np.array(direction)), color="green")

        print(f"{int(duration.secs) + (int(duration.nsecs) / 10e8):0.2f}s")

    ani = animation.FuncAnimation(fig=fig, func=update, frames=len(timestamps)//iteration_rate, interval=0)
    ani.save(f'/home/quentin/Dev/argos3-ros-triangulation/src/experiment_statistics/output/experiment_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.gif', writer='imagemagick', fps=24)
