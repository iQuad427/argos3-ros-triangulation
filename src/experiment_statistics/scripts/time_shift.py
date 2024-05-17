import copy
import math
import os

import numpy as np
import rosbag
import rospy

from utils import rotate_and_translate, euler_from_quaternion
import matplotlib.pyplot as plt


if __name__ == '__main__':
    input_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_launch/output"
    # experiment = "exp_11-landmarked-batch-1"
    experiment = "exp_12-landmarked-batch-2"
    input_bags = []

    mse_position = False

    directions = ["displacement"]
    # directions = ["displacement", "distances", "particles"]
    historic_size = [10]
    # historic_size = [1, 5, 10, 15, 20, 40]

    # time_shifts = [0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10]
    time_shifts = [6.5]

    time_shift_error = []

    inits = [False]
    # inits = [False, True]
    offsets = [True]
    # offsets = [False, True]
    certainties = [False]
    # certainties = [False, True]

    mds = False
    pf = False
    trilateration = True

    n_particles = [5000]
    agents_speed = 30
    sensor_std_errs = [10]
    dts = [0.1]

    if mds:
        for direction in directions:
            for size in historic_size:
                for init in inits:
                    for offset in offsets:
                        for certainty in certainties:
                            input_bags.append(
                                f"mds/mds{'_init' if init else ''}{'_offset' if offset else ''}{'_certainty' if certainty else ''}/seed_124_direction_{direction}_historic_{size}.bag"
                            )

    if pf:
        for direction in directions:
            for size in historic_size:
                for init in inits:
                    for offset in offsets:
                        for certainty in certainties:
                            for n_particle in n_particles:
                                for sensor_std_err in sensor_std_errs:
                                    for dt in dts:
                                        input_bags.append(
                                            f"pf_particles_{n_particle}_std_{sensor_std_err}_dt_{dt}/pf{'_init' if init else ''}{'_offset' if offset else ''}{'_certainty' if certainty else ''}/seed_124_direction_{direction}_historic_{size}.bag"
                                        )

    if trilateration:
        for direction in directions:
            for size in historic_size:
                for init in inits:
                    for offset in offsets:
                        for certainty in certainties:
                            input_bags.append(
                                f"trilateration/trilateration{'_init' if init else ''}{'_offset' if offset else ''}{'_certainty' if certainty else ''}/seed_124_direction_{direction}_historic_{size}.bag"
                            )

    topics = [
        "/compute/positions",
        "/experiment/positions"
    ]

    topics_to_shift = [
        "/experiment/positions"
    ]

    experiments = [
        f"{input_directory}/{experiment}/{input_bag}" for input_bag in input_bags
    ]

    for input_bag in experiments:
        for time_shift in time_shifts:
            dict_data = {
                topic: [] for topic in topics
            }

            timestamps = []
            real_positions = []
            estimated_positions = []

            # Create a temporary file
            os.system(f"cp {input_bag} /tmp/bag.bag")

            # Generate a new bag that shifts the time of the experiment
            with rosbag.Bag("/tmp/bag.bag", 'w') as bag:
                for topic, msg, t in rosbag.Bag(input_bag).read_messages():
                    if topic in topics_to_shift:
                        new_time = rospy.Time.from_sec(t.to_sec() + time_shift)
                        bag.write(topic, msg, new_time)
                    else:
                        bag.write(topic, msg, t)

            for topic, msg, t in rosbag.Bag("/tmp/bag.bag").read_messages():
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

            print(len(timestamps))

            # Compute the Mean Squared Error

            # Timestamp limited to duration of 120 seconds
            timestamps_new = []
            for timestamp in timestamps:
                if (timestamp - timestamps[0]).secs < 130:
                    timestamps_new.append(timestamp)

            timestamps = copy.deepcopy(timestamps_new)

            timesteps = []
            mean_squared_errors = []

            # Actual computation
            for i, timestamp in enumerate(timestamps):
                real_position = real_positions[i]
                estimated_position = estimated_positions[i]

                real_embedding = np.stack(
                    [(
                        p.pose.position.x * 100,
                        p.pose.position.y * 100,
                        euler_from_quaternion(
                            p.pose.orientation.x,
                            p.pose.orientation.y,
                            p.pose.orientation.z,
                            p.pose.orientation.w
                        )[2]
                    ) for p in real_position.positions]
                )

                estimated_embedding = np.stack(
                    [(
                        p.pose.position.x,
                        p.pose.position.y,
                        euler_from_quaternion(
                            p.pose.orientation.x,
                            p.pose.orientation.y,
                            p.pose.orientation.z,
                            p.pose.orientation.w
                        )[2]
                    ) for p in estimated_position.positions]
                )

                directions_sim = np.array([[np.cos(angle), np.sin(angle)] for angle in real_embedding[:, 2]])
                directions_est = np.array([[np.cos(angle), np.sin(angle)] for angle in estimated_embedding[:, 2]])

                corrected_embedding, corrected_directions, flip = rotate_and_translate(
                    real_embedding[:, 0:2], estimated_embedding[:, 0:2], directions=directions_est
                )

                if mse_position:
                    mse = np.mean(np.linalg.norm(real_embedding[:, 0:2] - corrected_embedding, axis=1))
                else:
                    # Come back to an angle from direction_est
                    direction = directions_est[2]
                    angle_estimated = math.atan2(
                        direction[1], direction[0]
                    )

                    mse = np.abs(real_embedding[2, 2] * 180/np.pi % 360 - angle_estimated * 180/np.pi % 360)
                    mse = mse if mse < 180 else mse - 360

                    # Take the absolute value
                    mse = np.abs(mse)

                timesteps.append((timestamp - timestamps[0]).secs + (timestamp - timestamps[0]).nsecs * 1e-9)
                mean_squared_errors.append(mse)

            # Look-up table to sort by time
            timesteps, mean_squared_errors = zip(*sorted(zip(timesteps, mean_squared_errors)))
            print(np.mean(mean_squared_errors))
            time_shift_error.append(np.mean(mean_squared_errors))

            label = f"{' '.join(input_bag.split('/')[-2].split('_'))}"
            # Add MSE to label if mse_positions is True
            # label += f" - MSE: {np.mean(mean_squared_errors):.2f} cm²" if mse_position else f" - MSE: {np.mean(mean_squared_errors):.2f}°"
            # Add direction and historic size to label if mse_positions is False
            if not mse_position:
                label += f" - {input_bag.split('/')[-1].split('_')[-3]} - {input_bag.split('/')[-1].split('_')[-1].split('.')[0]}"
            plt.plot(timesteps, mean_squared_errors, label=label)

            # Count the number of times mean_squared_errors is smaller than 30 and divide by the total number of elements
            print(f"Percentage of elements smaller than 30: {sum([1 for mse in mean_squared_errors if mse < 30]) / len(mean_squared_errors) * 100:.2f}%")

    if mse_position:
        plt.title("Mean Squared Error of Positions (Comparison between MDS and PF)")
        plt.xlabel("Time (s)")
        plt.ylabel("MSE (cm²)")
    else:
        plt.title("Error on Direction")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (°)")

    plt.legend()
    plt.savefig(f"{input_directory}/plot/{'mse_positions' if mse_position else 'mse_direction'}.png", dpi=300)
    plt.show()

    if len(time_shifts) > 5:
        plt.figure()
        plt.title("Average Mean Square Error over Time Shifts")
        plt.xlabel("Time Shift (s)")
        plt.ylabel("Average MSE (cm²)")
        plt.plot(time_shifts, time_shift_error)
        plt.show()
