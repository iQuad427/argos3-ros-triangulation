#!/usr/bin/python3

import copy
import dataclasses
import math
import pickle
import sys
import time
from collections import defaultdict
from datetime import datetime

import numpy as np
import rospy
import matplotlib.pyplot as plt

from morpho_msgs.msg import Direction, Angle, RangeAndBearing
from sklearn.linear_model import LinearRegression
from tri_msgs.msg import Distances, Distance, Odometry, Statistics

from utils import find_rotation_matrix, MultiAgentParticleFilter
from direction import find_direction_vector_from_position_history, range_and_bearing, generate_weighted_vector

from sklearn.manifold import MDS
import matplotlib
import matplotlib.backends.backend_agg as agg
import pylab
import pygame
from pygame.locals import *

import warnings

warnings.filterwarnings("ignore")

np.set_printoptions(linewidth=np.inf)
matplotlib.use("Agg")
pygame.init()

fig = pylab.figure(figsize=[6, 6], dpi=100)
ax = fig.gca()

canvas = agg.FigureCanvasAgg(fig)
renderer = canvas.get_renderer()

pygame.display.set_caption("Relative Trilateration of Swarm")

window = pygame.display.set_mode((600, 600), DOUBLEBUF)
screen = pygame.display.get_surface()

clock = pygame.time.Clock()

# Distance Measurements
distance_matrix = None
certainty_matrix = None

# Uncertainty on agents
last_update = None

# Running
running = False

# dict
hist_dist = defaultdict(list)

position_history = []
direction_history = []


def add_position(positions):
    global position_history
    position_history.insert(0, (datetime.now().timestamp(), positions))
    position_history = position_history[:3]
    return position_history


def add_direction(direction):
    global direction_history
    direction_history.insert(0, direction)
    direction_history = direction_history[:3]
    return direction_history


def correlated_positions(n):
    global position_history

    if len(position_history) < 2:
        return position_history[0][1]

    out_positions = []

    for i in range(n):
        times = np.array([entry[0] for entry in position_history]).reshape(-1, 1)
        positions = np.array([entry[1][i] for entry in position_history])

        model = LinearRegression()
        model.fit(times, positions)

        current_time = datetime.now().timestamp()
        next_time = current_time
        next_position = model.predict([[next_time]])

        out_positions.append(next_position[0])

    return out_positions


def add_for(x, y, dist_time):
    global hist_dist
    # COMMENT: - maxer l'estimation
    hist_dist[(x, y)] = [dist_time, *hist_dist[(x, y)]][:3]
    hist_dist[(y, x)] = [dist_time, *hist_dist[(y, x)]][:3]


# Function to build the distance matrix using linear regression
def build_distance_matrix(n, ):
    global hist_dist
    matrix = np.zeros(shape=(n, n))
    for (x, y), distances in copy.deepcopy(hist_dist).items():
        if len(distances) == 0:  # No data available
            continue

        # If there's only one data point, use it as the current distance
        if len(distances) == 1:
            current_distance = distances[0][0]
        else:
            # Extract features (time steps) and target (distances)
            X = [i[1] for i in distances]
            X = np.reshape(np.array(X), newshape=(-1, 1))
            Y = [i[0] for i in distances]

            # Fit linear regression model
            model = LinearRegression().fit(X, Y)

            # Predict the current value (distance)
            current_distance = model.predict([[datetime.now().timestamp()]])[0]

        # Update the distance matrix
        matrix[(x, y)] = current_distance
        matrix[(y, x)] = current_distance

    return matrix


def compute_positions(distances, multi_agent_pf, config=None):
    # Predict
    multi_agent_pf.predict(u=(0, config.agents_speed), std=(6.28, 5), dt=min((config.last_time - datetime.now()).seconds, config.dt))
    config.last_time = datetime.now()

    # Update
    z = np.copy(distances)  # Z is the distance matrix of real agents
    multi_agent_pf.update(z, config.sensor_std_err, multi_agent_pf.estimate())

    # Resample
    if multi_agent_pf.neff() < multi_agent_pf.N / 2:
        multi_agent_pf.resample()

    return multi_agent_pf.estimate()


def update_plot(agent, distances, embedding, historic):
    if distances is None:
        raise ValueError("Distance matrix should be defined at this point")

    if embedding is None:
        return

    # Reset the axes
    ax.clear()

    # Set the axes labels and title (customize as needed)
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('Particle Filter Plot')

    # Set the axes limits (customize as needed)
    ax.set_xlim(-600, 600)
    ax.set_ylim(-600, 600)

    # Put grid on the plot
    ax.grid(color='grey', linestyle='-', linewidth=0.1)

    # Update the scatter plot data
    plt.scatter(embedding[:, 0], embedding[:, 1], c='red')
    plt.scatter(embedding[agent, 0], embedding[agent, 1], c='blue')

    for i, point in enumerate(reversed(historic)):
        plt.scatter(point[0], point[1], color="r", alpha=0.5 / (i + 1))

    if historic:
        # Smoothing of direction
        direction_vector = find_direction_vector_from_position_history(historic) * 10
        direction_vector = generate_weighted_vector(add_direction(direction_vector))

    # Plot direction from current embedding of agent
    # plt.arrow()

    # Redraw the canvas
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()

    # Update the display
    size = canvas.get_width_height()
    surf = pygame.image.fromstring(raw_data, size, "RGB")
    screen.blit(surf, (0, 0))
    pygame.display.flip()


def add_distance(robot_idx, data: Distance):
    """
    :param robot_idx: the robot that measured the distance
    :param data: the distance it measured, with the certainty
    :return: Nothing, only parse to add to the distance_matrix
    """
    global distance_matrix, certainty_matrix

    if distance_matrix is None or last_update is None:
        raise ValueError("Distance matrix, certainty matrix and update table should be created at this point")

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
        add_for(x, y, (data.distance, datetime.now().timestamp()))


def callback(data, args):
    global last_update, running

    running = True

    if last_update is None:
        raise ValueError("Update table should be created at this point")

    if isinstance(data, Distance):
        self_idx = args[0] - ord('A')
        last_update[self_idx] = 0  # TODO: not sure that it should be updated this way

        add_distance(self_idx, data)
    elif isinstance(data, Distances):
        robot_idx = data.robot_id - ord('A')  # FIXME: Should start from 'B' since 'A' is the broadcast address
        last_update[robot_idx] = 0

        for robot in data.ranges:
            add_distance(robot_idx, robot)


def create_matrix(n: int):
    global distance_matrix, certainty_matrix, last_update

    # Create distance matrix (upper triangle cells are ones)
    mask = np.triu_indices(n, k=1)
    first_matrix = np.zeros((n, n))
    second_matrix = np.zeros((n, n))

    first_matrix[mask] = 1
    distance_matrix = first_matrix

    second_matrix[mask] = 1
    certainty_matrix = second_matrix

    # Create last update
    last_update = np.zeros((n,))

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError("Couldn't create distance matrix and/or certainty matrix")


@dataclasses.dataclass
class Config:
    n_particles: int
    agents_speed: float
    sensor_std_err: float
    dt: float
    last_time: datetime.now()


def listener():
    global distance_matrix, certainty_matrix, last_update, running

    # Parse arguments
    agent_id = sys.argv[1]

    # Parse arguments
    self_id = ord(agent_id[2])
    n_robots = int(sys.argv[2])

    create_matrix(n_robots)

    config = Config(
        n_particles=5000,
        agents_speed=30,
        sensor_std_err=30,
        dt=0.5,
        last_time=datetime.now()
    )

    multi_agent_pf = MultiAgentParticleFilter(n_robots, config.n_particles)

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError(
            "Distance matrix should exist at this point, ensure that you called create_matrix() beforehand")

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(f'/{agent_id}/distances', Distances, callback, (self_id,))
    rospy.Subscriber(f'/{agent_id}/distance', Distance, callback, (self_id,))
    pub = rospy.Publisher(f'/{agent_id}/range_and_bearing', RangeAndBearing, queue_size=10)
    statistics_pub = rospy.Publisher(f'/{agent_id}/positions', Statistics, queue_size=10)

    historic = []

    # Save previous values
    previous_estimation = compute_positions(
        (distance_matrix + distance_matrix.T),
        multi_agent_pf,
        config=config
    )  # Current estimation of the positions

    count = 0

    update_plot(self_id - ord('A'), distance_matrix, previous_estimation, historic)

    iteration_rate = 30
    crashed = False
    while not crashed:

        for event in pygame.event.get():
            if event.type == pygame.QUIT or rospy.is_shutdown():
                crashed = True

        if not running:
            clock.tick(iteration_rate)  # Limit frames per second
            continue

        # Update the data in the plot
        new_dm = distance_matrix + distance_matrix.T
        position_estimation = compute_positions(new_dm, multi_agent_pf, config=config)

        # Smoothing of agents positions
        # add_position(position_estimation)
        # corr_position = correlated_positions(n_robots)

        # Update position historic
        historic.append(list(position_estimation[self_id - ord('A')]))
        historic = historic[-5:]

        # Update the plot
        update_plot(self_id - ord('A'), new_dm, previous_estimation, historic)

        # Save current estimation
        previous_estimation = np.copy(position_estimation)

        # Save the data for later stats
        statistics_msg = Statistics()
        statistics_msg.header.stamp = rospy.Time.from_sec(datetime.now().timestamp())
        # print(statistics_msg.header.stamp)
        statistics_msg.header.frame_id = f"agent_{agent_id}"

        for i, position in enumerate(position_estimation):
            # print(i)
            # print(position)

            odometry_data = Odometry(
                id=i,
                x=position[0],
                y=position[1],
            )

            statistics_msg.odometry_data.append(odometry_data)

        statistics_pub.publish(statistics_msg)

        # Tick for uncertainty increase
        last_update = last_update + 1
        certainty_matrix = certainty_matrix * 0.99

        # Tick the update clock
        clock.tick(iteration_rate)  # Limit to X frames per second
        count += 1


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
