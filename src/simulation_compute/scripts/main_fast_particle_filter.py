#!/usr/bin/python3

import dataclasses
import signal
import sys
import warnings
from collections import defaultdict
from datetime import datetime

import numpy as np
import pygame
import rospy
from simulation_utils.msg import Distances, Distance, Odometry, Positions, Manage

from utils import MultiAgentParticleFilter, compute_direction, euler_to_quaternion

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


def compute_positions(distances, certainties, multi_agent_pf, config=None):
    # Predict
    multi_agent_pf.predict(u=(0, config.agents_speed), std=(6.28, 5), dt=min((config.last_time - datetime.now()).seconds, config.dt))
    config.last_time = datetime.now()

    matrix = np.copy(distances)

    # Update
    if config.offset:
        # Remove 20 to all values
        matrix = matrix - 20

        # 0 on the diagonal
        np.fill_diagonal(matrix, 0)

        # Negative values to 0
        matrix = np.where(matrix < 0, 0, matrix)

    if config.certainty:
        matrix[certainties < 0.5] = -1

    z = np.copy(matrix)
    multi_agent_pf.update(z, config.sensor_std_err, multi_agent_pf.estimate())

    # Resample
    if multi_agent_pf.neff() < multi_agent_pf.N / 2:
        multi_agent_pf.resample()

    return multi_agent_pf.estimate()


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

    first_matrix[mask] = -1
    distance_matrix = first_matrix

    second_matrix[mask] = 0
    certainty_matrix = second_matrix

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError("Couldn't create distance matrix and/or certainty matrix")


@dataclasses.dataclass
class Config:
    # Experiment
    random_seed: int
    init: bool
    offset: bool
    certainty: bool

    # Particle Filter
    n_particles: int
    agents_speed: float
    sensor_std_err: float
    dt: float
    last_time: datetime.now()


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
        n_particles=int(sys.argv[7]),
        agents_speed=int(sys.argv[8]),
        sensor_std_err=int(sys.argv[9]),
        dt=float(sys.argv[10]),
        last_time=datetime.now(),
    )

    np.random.seed(config.random_seed)

    print("Running with the following configuration:")
    print(config)

    create_matrix(n_robots)

    multi_agent_pf = MultiAgentParticleFilter(
        n_robots,
        config.n_particles,
        initial_poses=[
            [0, 0, 0],
            [30, 0, 0],
            [0, 30, 0],
            [30, 30, 0]
        ] if config.init else None
    )

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError("Distance matrix should exist at this point, ensure that you called create_matrix() beforehand")

    rospy.init_node('pf_compute', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    manage_publisher = rospy.Publisher('simulation/manage_command', Manage, queue_size=10)
    # rospy.Subscriber('simulation/manage_command', Manage, callback)

    historical_data = defaultdict(list)

    rospy.Subscriber(f'/{agent_id}/distances', Distances, lambda data: historical_data[data.timestamp].append(data), queue_size=10000)
    statistics_pub = rospy.Publisher(f'/{agent_id}/positions', Positions, queue_size=10000)

    start = datetime.now()

    while not rospy.is_shutdown() and (datetime.now() - start).total_seconds() < 10 and not stop:
        pass

    print("START COMPUTING")

    # Start time is smallest timestep in the simulation data
    start_time = min(historical_data.keys())

    embedding_historic = []
    positions_historic = []
    distances_historic = []

    # Save previous values
    previous_estimation = compute_positions(
        (distance_matrix + distance_matrix.T),
        (certainty_matrix + certainty_matrix.T),
        multi_agent_pf,
        config=config
    )  # Current estimation of the positions

    for step in sorted(historical_data.keys()):
        if stop:
            break

        # Update using data received at timestep
        distances = historical_data[step]

        if len(distances) == 0:
            continue

        # Update the distance matrix with current information
        for distance in distances:
            sensor_callback(distance, (self_idx,))

        # Tick for uncertainty increase
        certainty_matrix = certainty_matrix * 0.99

        # Should not compute every millisecond, even if the simulation is that fast
        should_compute = int((step - start_time) * 100) % 5 == 0
        if not should_compute:
            continue

        # Distance matrix should be symmetrical
        new_dm = distance_matrix + distance_matrix.T
        new_certainty = certainty_matrix + certainty_matrix.T

        # Update the data in the plot
        position_estimation = compute_positions(
            new_dm,
            new_certainty,
            multi_agent_pf,
            config=config
        )

        # Update position historic
        positions_historic.append(list(position_estimation[self_idx - ord('A')]))
        positions_historic = positions_historic[-5:]

        embedding_historic.append(np.copy(position_estimation))
        embedding_historic = embedding_historic[-5:]

        distances_historic.append(np.copy(new_dm[0]))
        distances_historic = distances_historic[-5:]

        distance_direction, historic_direction, particle_direction = compute_direction(
            embedding_historic,
            positions_historic,
            distances_historic
        )

        # Save the data for later stats
        positions_msg = Positions()
        positions_msg.timestamp = step - start_time

        # Compute angle of the robot from the direction vector
        direction = historic_direction
        if direction is not None:
            direction_angle = np.arctan2(direction[1], direction[0])
        else:
            direction_angle = 0

        for i, position in enumerate(position_estimation):
            if i != self_idx - ord('A'):
                angle = (0, 0, 0, 1)
            else:
                angle = euler_to_quaternion(0, 0, direction_angle)

            odometry = Odometry(
                id=i,
                x=position[0],
                y=position[1],
                z=0,
                # TODO: add the orientation information for all agents ?
                a=angle[0],
                b=angle[1],
                c=angle[2],
                d=angle[3]
            )

            positions_msg.odometry_data.append(odometry)

        statistics_pub.publish(positions_msg)

    print("STOP COMPUTING")

    manage_publisher.publish(Manage(stop=True))


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
