#!/usr/bin/python3
import dataclasses
import signal
import sys
import warnings
from datetime import datetime

import numpy as np
import pygame
import rospy
from experiment_utils.msg import Distances, Distance, Odometry, Positions, Manage
from geometry_msgs.msg import Pose, Point, Quaternion
from sklearn.manifold import MDS

from utils import compute_direction, euler_to_quaternion, DirectionsEnum

warnings.filterwarnings("ignore")

pygame.init()

clock = pygame.time.Clock()

# Distance Measurements
distance_matrix = None
certainty_matrix = None

# Running
running = False
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
    directions: DirectionsEnum

    def __repr__(self):
        return f"Config(random_seed={self.random_seed}, init={self.init}, offset={self.offset}, certainty={self.certainty})"


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

    other_robot_idx = data.other_robot_id - ord('B')

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
        self_idx = args[0] - ord('B')
        add_distance(self_idx, data)
    elif isinstance(data, Distances):
        robot_idx = data.robot_id - ord('B')

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
    global distance_matrix, certainty_matrix, running

    # Parse arguments
    agent_id = rospy.get_param("b")
    self_idx = ord(agent_id[2])

    # Parse arguments
    n_robots = rospy.get_param("n")

    config = Config(
        random_seed=rospy.get_param("random_seed"),
        init=rospy.get_param("init"),
        offset=rospy.get_param("offset"),
        certainty=rospy.get_param("certainty"),
        directions=DirectionsEnum(rospy.get_param("directions"))
    )

    print("Running with the following configuration:")
    print(config)

    # Parse beacons
    if rospy.get_param('beacons') != "Z":
        beacons = [ord(beacon) - ord('B') for beacon in rospy.get_param('beacons').split(",")]
    else:
        beacons = None

    create_matrix(n_robots)

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError(
            "Distance matrix should exist at this point, ensure that you called create_matrix() beforehand")

    rospy.init_node('mds_compute', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    rospy.Subscriber('experiment/manage_command', Manage, callback)

    rospy.Subscriber(f'/init/sensor_read', Distances, sensor_callback, (self_idx,))
    rospy.Subscriber(f'/resp/sensor_read', Distances, sensor_callback, (self_idx,))
    positions_pub = rospy.Publisher(f'/compute/positions', Positions, queue_size=10)

    embedding_historic = []
    positions_historic = []
    distances_historic = []

    # Save previous values
    previous_estimation = None  # Positions used to rotate the plot (avoid flickering when rendering in real time)
    previous_estimation = compute_positions(
        (distance_matrix + distance_matrix.T),
        certainty_matrix,
        previous_estimation,
        beacons=beacons,
        config=config
    )  # Current estimation of the positions

    iteration_rate = 30
    crashed = False
    while not crashed and not stop:

        for event in pygame.event.get():
            if event.type == pygame.QUIT or rospy.is_shutdown():
                crashed = True

        if not running:
            clock.tick(iteration_rate)  # Limit frames per second
            continue

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

        # Update position historic
        positions_historic.append(list(position_estimation[self_idx - ord('B')]))
        positions_historic = positions_historic[-5:]

        embedding_historic.append(np.copy(position_estimation))
        embedding_historic = embedding_historic[-5:]

        distances_historic.append(np.copy(new_dm[0]))
        distances_historic = distances_historic[-5:]

        if config.directions == DirectionsEnum.DISTANCES:
            pass
        elif config.directions == DirectionsEnum.DISPLACEMENT:
            pass
        elif config.directions == DirectionsEnum.PARTICLES:
            pass

        distance_direction, historic_direction, particle_direction = compute_direction(
            embedding_historic,
            positions_historic,
            distances_historic
        )

        # Save current estimation
        previous_estimation = np.copy(position_estimation)

        # Save the data for later stats
        positions_msg = Positions()
        positions_msg.header.stamp = rospy.Time.now()

        # Compute angle of the robot from the direction vector
        direction = distance_direction
        if direction is not None:
            direction_angle = np.arctan2(direction[1], direction[0])
        else:
            direction_angle = 0

        for i, position in enumerate(position_estimation):
            if i != self_idx - ord('B'):
                angle = (0, 0, 0, 1)
            else:
                angle = euler_to_quaternion(0, 0, direction_angle)

            odometry = Odometry(
                id=i,
                pose=Pose(
                    position=Point(
                        x=position[0],
                        y=position[1],
                        z=0,
                    ),
                    orientation=Quaternion(
                        x=angle[0],
                        y=angle[1],
                        z=angle[2],
                        w=angle[3]
                    )
                )
            )

            positions_msg.positions.append(odometry)

        positions_pub.publish(positions_msg)

        # Tick for uncertainty increase
        certainty_matrix = certainty_matrix * 0.99

        # Tick the update clock
        clock.tick(iteration_rate)  # Limit to X frames per second


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
