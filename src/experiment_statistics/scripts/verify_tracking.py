import copy
import datetime

import numpy as np
import rosbag
from matplotlib import animation

from utils import euler_from_quaternion

if __name__ == '__main__':
    input_bag = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_utils/output/exp_5-one_moving_random-working_sensors/complete_experiment.bag"
    saved_positions = None

    topics = [
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

                # Save in lists
                timestamps.append(t)
                real_positions.append(real)

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

    fig, ax = plt.figure(figsize=(10, 5))

    # Top title
    plt.suptitle(f"Timestamp: 0s")

    ax.set_title("Real positions")
    ax.set(xlim=[-3, 3], ylim=[-3, 3], xlabel='X [m]', ylabel='Y [m]')

    real_plot = ax.scatter(
        *np.stack(
            [(p.pose.position.x, p.pose.position.y) for p in real_positions[0].positions]
        ).T
    )

    iteration_rate = 5

    def update(frame):
        ax.clear()

        ax.set(xlim=[-3, 3], ylim=[-3, 3], xlabel='X [m]', ylabel='Y [m]')

        frame = frame * iteration_rate

        timestamp = timestamps[frame]
        real_position = real_positions[frame]

        duration = (timestamp - timestamps[0])

        if duration.secs > 130:
            return

        plt.suptitle(f"Timestamp: {int(duration.secs) + (int(duration.nsecs) / 10e8):0.2f}s")

        real_plot.set_offsets(
            np.stack(
                [(p.pose.position.x, p.pose.position.y) for p in real_position.positions]
            )
        )

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

        print(f"{int(duration.secs) + (int(duration.nsecs) / 10e8):0.2f}s")

        return real_plot

    ani = animation.FuncAnimation(fig=fig, func=update, frames=len(timestamps)//iteration_rate, interval=0)
    ani.save(f'/home/quentin/Dev/argos3-ros-triangulation/src/experiment_statistics/output/experiment_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.gif', writer='imagemagick', fps=24)
