#!/usr/bin/python3
import dataclasses
import datetime
import signal
from typing import List

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy
import sys

from tri_msgs.msg import Statistics, Odometry

from plot_2 import Position, Memory, FileReader
from utils import rotate_and_translate

iterate = False
compute = True


# Add handler for SIGINT
def signal_handler(sig, frame):
    global compute
    compute = False
    print("Exiting...")


def positions_callback(positions, args):
    timestamp = datetime.datetime.utcfromtimestamp(positions.header.stamp.to_sec())

    storage = args[0]

    memory = []
    for position in positions.odometry_data:
        memory.append(
            Position(
                position.id if position.id < ord('A') else position.id - ord('A'),
                position.x if args[1] == "agent" else position.x * 100,
                position.y if args[1] == "agent" else position.y * 100,
            )
        )

    storage.append(Memory(memory, timestamp))

    if args[1] == "agent":
        global iterate
        iterate = True

    # Keep only the first and last one (use reference to list)
    storage = storage[-1:] + storage[:0]


def listener():
    global iterate

    output_dir = sys.argv[1]
    output_file = sys.argv[2]

    number_agents = sys.argv[3]
    simulation_positions, estimated_positions = [], []

    # Generate one memory for each list
    simulation_positions.append(Memory([
        Position(i, 0, 0) for i in range(int(number_agents))
    ], datetime.datetime.now()))

    # estimated_positions.append(Memory([
    #     Position(i, 0, 0) for i in range(int(number_agents))
    # ], datetime.datetime.now()))

    agent_id = sys.argv[4]
    print(f'Agent ID: {agent_id}')

    rospy.init_node('statistics', anonymous=True)

    experiment_date = datetime.datetime.now()

    # Listen to statistics ROS topic
    rospy.Subscriber(f'/simulation/positions', Statistics, positions_callback, (simulation_positions, "simulation"))
    rospy.Subscriber(f'/{agent_id}/positions', Statistics, positions_callback, (estimated_positions, "agent"))

    time_diff = []

    # Name of file is statistics_output_dd_mm_hh_mm.txt
    with open(f"{output_dir}/{output_file}", "w+") as f:
        # f.write("type,time,position\n")  # Write header for CSV file
        while compute:
            if iterate:
                ground_truth = simulation_positions[-1]
                estimation = estimated_positions[-1]

                start = estimated_positions[0].timestamp
                now = (estimation.timestamp - start).total_seconds()

                print(now)

                f.write(f"estim={now}={estimation}\n")
                f.write(f"truth={now}={ground_truth}\n")

                time_diff.append((estimation.timestamp - ground_truth.timestamp).total_seconds())

                iterate = False

    print("Total time :", simulation_positions[-1].timestamp - simulation_positions[0].timestamp)

    # Calculate the mean time difference
    mean_time_diff = np.mean(time_diff)
    print(f"Mean time difference: {mean_time_diff}")

    # Read the file
    file_reader = FileReader(f"{output_dir}/{output_file}")

    print("[0] File :", file_reader.file_path)
    print("[0] File name :", file_reader.file_name)

    mean_square_error = []

    for time_step in file_reader.data:
        estimation = time_step[0]
        simulation = time_step[1]

        # Generate numpy arrays for the positions
        pos_estimation = [[position.x, position.y] for position in estimation.positions]
        pos_simulation = [[position.x, position.y] for position in simulation.positions]

        pos_estimation = np.array(pos_estimation)
        # Generate flipped numpy array for the simulation positions (symmetry along x-axis)
        flipped_estimation = np.array([[position[0], -position[1]] for position in pos_estimation])

        pos_simulation = np.array(pos_simulation)

        # Rotate to best fit the simulation positions
        from utils import rotate_and_translate

        pos_estimation = rotate_and_translate(pos_simulation, pos_estimation)
        flipped_estimation = rotate_and_translate(pos_simulation, flipped_estimation)

        # Calculate the Mean Square Error
        mse_non_flipped = np.mean(np.square(pos_estimation - pos_simulation))
        mse_flipped = np.mean(np.square(flipped_estimation - pos_simulation))

        mean_square_error.append(min(mse_non_flipped, mse_flipped))

    # Axis labels
    plt.xlabel("Time (s)")
    plt.ylabel("Mean Square Error (cm²)")

    # Plot the Mean Square Error
    plt.plot(file_reader.time, mean_square_error)
    plt.show()


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Print current directory
    print(f"Current directory: {sys.path[0]}")

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
