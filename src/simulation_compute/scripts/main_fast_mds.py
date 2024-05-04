#!/usr/bin/python3
import dataclasses
import signal
import sys
import warnings
from collections import defaultdict
from datetime import datetime

import matplotlib
import numpy as np
import pygame
import rospy
from simulation_utils.msg import Distances, Distance, Odometry, Positions, Manage
from sklearn.manifold import MDS

warnings.filterwarnings("ignore")

# Distance Measurements
distance_matrix = None
certainty_matrix = None

stop = False


def callback(msg):
    global stop

    if msg.stop:
        stop = True


def signal_handler(sig, frame):
    global stop
    stop = True


@dataclasses.dataclass
class Config:
    random_seed: int
    init: bool
    offset: bool
    certainty: bool

    def __repr__(self):
        return f"Config(random_seed={self.random_seed}, init={self.init}, offset={self.offset}, certainty={self.certainty})"


def parse_distances(line: str) -> Distances:
    msg = Distances()
    msg.ranges = []

    faulty_frame = False

    infos = line.split(",")

    if len(infos) > 1:  # Means that the split was made (therefore at least one information to process)
        sender_robot = infos[0]
        sender_info = sender_robot.split("=")

        if sender_info[1] == "0:100":
            msg.robot_id = ord(sender_info[0])

        for info in infos[1:]:
            try:
                robot, distance = info.split("=")
                distance, certainty = distance.split(":")

                data = Distance()

                data.other_robot_id = ord(robot)
                data.distance = int(distance)
                data.certainty = int(certainty)

                msg.ranges.append(data)
            except ValueError as e:
                pass

    return msg, faulty_frame


def parse_positions(line: str) -> Positions:
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


def compute_positions(distances, certainties, ref_plot, beacons=None, config=None):
    if distances is not None:
        # Update the data in the plot
        # Make sure the matrix is symmetric
        matrix = (distances + distances.T) / 2  # removed '/2' because triangular matrix
        matrix_certainty = (certainties + certainties.T)

        # Use sklearn MDS to reduce the dimensionality of the matrix
        mds = MDS(
            n_components=2,
            dissimilarity='precomputed',
            normalized_stress=False,
            metric=True,
            random_state=config.random_seed
        )

        if config.offset:
            # Remove 20 to all values
            matrix = matrix - 20

            # 0 on the diagonal
            np.fill_diagonal(matrix, 0)

            # Negative values to 0
            matrix = np.where(matrix < 0, 0, matrix)

        # print("Offset applied") if config.offset else print("No offset applied")

        weights = matrix_certainty if config.certainty else None
        # print(f"Using weights") if config.certainty else print(f"Not using weights")

        if config.init:
            # print("Initialized MDS")
            if ref_plot is None or ref_plot[(0, 0)] == .0:
                embedding = mds.fit_transform(matrix, weight=weights)
            else:
                try:
                    embedding = mds.fit_transform(matrix, weight=weights, init=ref_plot)
                except:
                    embedding = mds.fit_transform(matrix, weight=weights)
        else:
            # print("Not initialized MDS")
            embedding = mds.fit_transform(matrix, weight=weights)

        return embedding


def add_distance(robot_idx, data: Distance):
    """
    :param robot_idx: the robot that measured the distance
    :param data: the distance it measured, with the certainty
    :return: Nothing, only parse to add to the distance_matrix
    """
    global distance_matrix, certainty_matrix

    if distance_matrix is None:
        raise ValueError("Distance matrix and certainty matrix should be created at this point")

    other_robot_idx = data.other_robot_id - ord('A')

    x = robot_idx
    y = other_robot_idx
    if robot_idx > other_robot_idx:
        x = other_robot_idx
        y = robot_idx

    # Only update if certainty is greater than the one of the previous measurement
    if certainty_matrix[x, y] < data.certainty:
        distance_matrix[x, y] = data.distance
        certainty_matrix[x, y] = data.certainty


def sensor_callback(data, args):
    global running

    running = True

    if isinstance(data, Distance):
        self_idx = args[0] - ord('A')
        add_distance(self_idx, data)
    elif isinstance(data, Distances):
        robot_idx = data.robot_id - ord('A')

        for robot in data.ranges:
            add_distance(robot_idx, robot)


def create_matrix(n: int):
    global distance_matrix, certainty_matrix

    # Create distance matrix (upper triangle cells are ones)
    mask = np.triu_indices(n, k=1)
    first_matrix = np.zeros((n, n))
    second_matrix = np.zeros((n, n))

    first_matrix[mask] = 1
    distance_matrix = first_matrix

    second_matrix[mask] = 1
    certainty_matrix = second_matrix

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError("Couldn't create distance matrix and/or certainty matrix")


def listener():
    global distance_matrix, certainty_matrix

    # Parse arguments
    agent_id = sys.argv[1]
    self_idx = ord(agent_id[2])

    # Parse arguments
    n_robots = int(sys.argv[2])

    config = Config(
        random_seed=int(sys.argv[3]),
        init=True if sys.argv[4] == "True" else False,
        offset=True if sys.argv[5] == "True" else False,
        certainty=True if sys.argv[6] == "True" else False,
    )

    print("Running with the following configuration:")
    print(config)

    if sys.argv[7] != "Z":
        beacons = [ord(beacon) - ord('A') for beacon in sys.argv[3].split(",")]
    else:
        beacons = None

    create_matrix(n_robots)

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError(
            "Distance matrix should exist at this point, ensure that you called create_matrix() beforehand")

    rospy.init_node('mds_compute', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    manage_publisher = rospy.Publisher('simulation/manage_command', Manage, queue_size=10)
    # rospy.Subscriber('simulation/manage_command', Manage, callback)

    historical_data = defaultdict(list)

    rospy.Subscriber(f'/{agent_id}/distances', Distances, lambda data: historical_data[data.timestep].append(data), queue_size=2000)
    statistics_pub = rospy.Publisher(f'/{agent_id}/positions', Positions, queue_size=2000)

    start = datetime.now()

    while not rospy.is_shutdown() and (datetime.now() - start).total_seconds() < 5 and not stop:
        pass

    print("START COMPUTING")

    # Start time is smallest timestep in the simulation data
    start_time = min(historical_data.keys())

    # Save previous values
    previous_estimation = None  # Positions used to rotate the plot (avoid flickering when rendering in real time)
    previous_estimation = compute_positions(
        (distance_matrix + distance_matrix.T),
        certainty_matrix,
        previous_estimation,
        beacons=beacons,
        config=config
    )  # Current estimation of the positions

    for step in range(start_time, max(historical_data.keys())):
        if stop:
            break

        distances = historical_data[step]

        if len(distances) == 0:
            continue

        # Update the distance matrix with current information
        for distance in distances:
            sensor_callback(distance, (self_idx,))

        # Distance matrix should be symmetrical
        new_dm = distance_matrix + distance_matrix.T

        # Update the data in the plot
        position_estimation = compute_positions(
            new_dm,
            certainty_matrix,
            previous_estimation,
            beacons=beacons,
            config=config
        )

        # Save current estimation
        previous_estimation = np.copy(position_estimation)

        # Save the data for later stats
        statistics_msg = Positions()
        statistics_msg.timestep = step - start_time

        for i, position in enumerate(position_estimation):
            odometry_data = Odometry(
                id=i,
                x=position[0],
                y=position[1],
            )

            statistics_msg.odometry_data.append(odometry_data)

        statistics_pub.publish(statistics_msg)

        # Tick for uncertainty increase
        certainty_matrix = certainty_matrix * 0.99

    print("STOP COMPUTING")

    manage_publisher.publish(Manage(stop=True))


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass