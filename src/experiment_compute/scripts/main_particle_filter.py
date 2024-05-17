#!/usr/bin/python3

import dataclasses
import signal
import warnings

import numpy as np
import pygame
import rospy
from experiment_utils.msg import Distances, Distance, Odometry, Positions, Manage
from geometry_msgs.msg import Pose, Point, Quaternion

from utils import MultiAgentParticleFilter, euler_to_quaternion
from direction import compute_direction, DirectionsEnum

warnings.filterwarnings("ignore")

pygame.init()

clock = pygame.time.Clock()

# Distance Measurements
distance_matrix = None
certainty_matrix = None

beacons_positions = {}

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


def compute_positions(distances, certainties, multi_agent_pf, config=None):
    # Predict
    multi_agent_pf.predict(u=(0, config.agents_speed), std=(6.28, 5), dt=config.dt)
    # config.last_time = rospy.Time.now()

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

    other_robot_idx = data.other_robot_id

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
        robot_idx = data.robot_id

        for robot in data.ranges:
            add_distance(robot_idx, robot)


def beacons_callback(data):
    global beacons_positions

    for odometry in data.positions:
        element = odometry.pose.position
        element.x *= 100
        element.y *= 100
        beacons_positions[odometry.id] = element


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
    directions: DirectionsEnum
    historic_size: int

    # Particle Filter
    n_particles: int
    agents_speed: float
    sensor_std_err: float
    dt: float
    last_time: rospy.rostime.Time


def listener():
    global distance_matrix, certainty_matrix, running

    # Parse arguments
    agent_id = rospy.get_param("id")
    self_idx = ord(agent_id[2])

    # Parse arguments
    n_robots = rospy.get_param("n")

    rospy.init_node('pf_compute', anonymous=True)

    config = Config(
        random_seed=rospy.get_param("random_seed"),
        init=rospy.get_param("init"),
        offset=rospy.get_param("offset"),
        certainty=rospy.get_param("certainty"),
        directions=DirectionsEnum(rospy.get_param("directions")),
        historic_size=rospy.get_param("historic_size"),
        n_particles=rospy.get_param("n_particles"),
        agents_speed=rospy.get_param("agents_speed"),
        sensor_std_err=rospy.get_param("sensor_std_err"),
        dt=rospy.get_param("dt"),
        last_time=rospy.Time.now(),
    )

    np.random.seed(config.random_seed)

    print("Running with the following configuration:")
    print(config)

    create_matrix(n_robots)

    multi_agent_pf = MultiAgentParticleFilter(
        n_robots,
        config.n_particles,
        # TODO: initialization is not really implemented here
        initial_poses=[
            [0, 0, 0],
            [30, 0, 0],
            [0, 30, 0],
            [30, 30, 0]
        ] if config.init else None
    )

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError("Distance matrix should exist at this point, ensure that you called create_matrix() beforehand")

    # Subscribe to the manager command (to stop the node when needed)
    rospy.Subscriber('experiment/manage_command', Manage, callback)

    rospy.Subscriber(f'/init/sensor_read', Distances, sensor_callback, (self_idx,))
    rospy.Subscriber(f'/resp/sensor_read', Distances, sensor_callback, (self_idx,))
    rospy.Subscriber(f'/experiment/positions', Positions, beacons_callback)
    positions_pub = rospy.Publisher(f'/compute/positions', Positions, queue_size=10)

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

    iteration_rate = 30
    while not stop:
        if not running:
            clock.tick(iteration_rate)  # Limit frames per second
            continue

        # Update the data in the plot
        new_dm = distance_matrix + distance_matrix.T
        new_certainty = certainty_matrix + certainty_matrix.T
        position_estimation = compute_positions(new_dm, new_certainty, multi_agent_pf, config=config)

        # Update position historic
        size = config.historic_size

        positions_historic.append(list(position_estimation[self_idx - ord('B')]))
        positions_historic = positions_historic[-size:]

        embedding_historic.append(np.copy(position_estimation))
        embedding_historic = embedding_historic[-size:]

        distances_historic.append(np.copy(new_dm[0]))
        distances_historic = distances_historic[-size:]

        direction = compute_direction(
            embedding_historic,
            positions_historic,
            distances_historic,
            direction_type=config.directions
        )

        # Save current estimation
        previous_estimation = np.copy(position_estimation)

        # Save the data for later stats
        positions_msg = Positions()
        positions_msg.header.stamp = rospy.Time.now()

        # Compute angle of the robot from the direction vector
        if np.linalg.norm(direction) > 0:
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
