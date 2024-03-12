#!/usr/bin/python3
import math
import pickle
import sys

import numpy as np
import rospy
import matplotlib.pyplot as plt

from morpho_msgs.msg import Direction, Angle
from tri_msgs.msg import Distances, Distance
from utils import find_rotation_matrix
from direction import generate_morph_msg

from sklearn.manifold import MDS
import matplotlib
import matplotlib.backends.backend_agg as agg
import pylab
import pygame
from pygame.locals import *


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

modified = False

# Uncertainty on agents
last_update = None


def compute_positions(distances, ref_plot, beacons=None):
    if distances is not None:
        matrix = distances

        # Update the data in the plot
        # Make sure the matrix is symmetric
        matrix = (matrix + matrix.T)  # removed '/2' because triangular matrix

        # Use sklearn MDS to reduce the dimensionality of the matrix
        mds = MDS(n_components=2, dissimilarity='precomputed', normalized_stress=False, metric=True, random_state=42)
        embedding = mds.fit_transform(matrix)

        # Rotate dots to match previous plot
        if ref_plot is not None:
            if beacons is None or len(beacons) < 2:
                rotation = find_rotation_matrix(ref_plot.T, embedding.T)
            else:
                rotation = find_rotation_matrix(ref_plot[beacons].T, embedding[beacons].T)

            # Apply the rotation
            embedding = embedding @ rotation

            if beacons is None:
                # First, find the centroid of the original points
                previous_centroid = np.mean(ref_plot, axis=0)
                # Then, find the centroid of the MDS points
                current_centroid = np.mean(embedding, axis=0)
            else:
                # TODO: beacons should be the list of IDs of the beacons (therefore, need to modify code)
                previous_centroid = np.mean(ref_plot[beacons], axis=0)
                current_centroid = np.mean(embedding[beacons], axis=0)

            # Find the translation vector
            translation = previous_centroid - current_centroid

            # Translate the MDS points
            embedding = embedding + translation

        return embedding


def update_plot(distances, embedding, measurement_uncertainty, time_uncertainty):
    if distances is None:
        raise ValueError("Distance matrix should be defined at this point")

    if embedding is None:
        return

    # Reset the axes
    ax.clear()

    # Set the axes labels and title (customize as needed)
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('MDS Scatter Plot')

    # Set the axes limits (customize as needed)
    ax.set_xlim(-600, 600)
    ax.set_ylim(-600, 600)

    # Put grid on the plot
    ax.grid(color='grey', linestyle='-', linewidth=0.1)

    # Update the scatter plot data
    plt.scatter(embedding[:, 0], embedding[:, 1], c='r')
    plt.scatter(embedding[0, 0], embedding[0, 1], c='b')
    plt.scatter(embedding[1, 0], embedding[1, 1], c='g')
    plt.scatter(embedding[2, 0], embedding[2, 1], c='y')

    # Make the triangle matrix symmetric to simplify data access
    distances = distances + distances.T

    for i, agent in enumerate(embedding):
        ax.add_patch(
            plt.Circle(
                agent,
                time_uncertainty[i],
                color='r', fill=False
            )
        )
        ax.add_patch(
            plt.Circle(
                agent,
                distances[0, i],
                color='b', fill=False,
                linewidth=10 / (1 - measurement_uncertainty[i]), alpha=0.1
            )
        )

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
    global distance_matrix, certainty_matrix, modified

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

        modified = True


def callback(data, args):
    global last_update

    if last_update is None:
        raise ValueError("Update table should be created at this point")

    # TODO: create uncertainty system, uncertainty increase on measurements received a long time ago,
    #       but also, when updated, choose the measurement with the least uncertainty

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


def compute_measurement_uncertainty(certainty):
    matrix = certainty + certainty.T

    # Compute mean certainty for each agent distances (each row or column)
    certainty = np.mean(matrix, axis=0)

    print(certainty)
    print(-certainty)

    return (-certainty + 100) / 100  # (uncertainty factor)


def compute_time_uncertainty(update, speed, error):
    return update * speed + error


def listener():
    global distance_matrix, certainty_matrix, modified, last_update

    # Parse arguments
    ros_launch_param = sys.argv[1]

    # Parse arguments
    self_id = ord(ros_launch_param[2])
    n_robots = int(sys.argv[2])

    if sys.argv[3] != "Z":
        beacons = [ord(beacon) - ord("A") for beacon in sys.argv[3].split(",")]
    else:
        beacons = None

    create_matrix(n_robots)

    if distance_matrix is None or certainty_matrix is None:
        raise ValueError("Distance matrix should exist at this point, ensure that you called create_matrix() beforehand")

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(f'/{ros_launch_param}/distances', Distances, callback, (self_id,))
    rospy.Subscriber(f'/{ros_launch_param}/distance', Distance, callback, (self_id,))

    data = []

    uncertainty = compute_measurement_uncertainty(certainty_matrix)

    # Save previous values
    previous_estimation = None  # Positions used to rotate the plot (avoid flickering when rendering in real time)
    position_estimation = compute_positions(distance_matrix, previous_estimation, beacons=beacons)  # Current estimation of the positions

    # Positions used for direction estimation
    current_plot = None  # Idea: might want to update less often to have more distance measurements updates

    with open("output/output.txt", "wb") as f:
        crashed = False
        while not crashed:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or rospy.is_shutdown():
                    crashed = True

            # Direction estimation
            measurement_uncertainty = compute_measurement_uncertainty(certainty_matrix)
            time_uncertainty = compute_time_uncertainty(last_update, 3, 10)

            print(measurement_uncertainty)
            print(time_uncertainty)

            update_plot(distance_matrix, previous_estimation, measurement_uncertainty, time_uncertainty)

            if modified:  # Only render and send message if data has changed
                # TODO: modification should only take place after noise mitigation processes.
                #       Here, it only indicates that new data was received from another agent.

                # Update the data in the plot
                position_estimation = compute_positions(distance_matrix, previous_estimation, beacons=beacons)
                modified = False

            # Save the data for later stats
            if position_estimation is not None:
                data.append(position_estimation)

            previous_estimation = np.copy(position_estimation)

            # Tick for uncertainty increase
            last_update = last_update + 1
            certainty_matrix = certainty_matrix * 0.99

            # Tick the update clock
            clock.tick(0.5)  # Limit to 30 frames per second

        pickle.dump(data, f)


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
