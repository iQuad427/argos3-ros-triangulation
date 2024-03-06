#!/usr/bin/python3
import math
import pickle
import sys

import numpy as np
import rospy
import matplotlib.pyplot as plt

from morpho_msgs.msg import Direction, Angle
from tri_msgs.msg import Distances, Distance
from utils import compute_direction, find_rotation_matrix
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

distance_matrix = None
distance_table = None
modified = False


def update_plot(distances, ref_plot):
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
            rotation = find_rotation_matrix(ref_plot.T, embedding.T)

            # Apply the rotation
            embedding = embedding @ rotation

            # First, find the centroid of the original points
            centroid = np.mean(ref_plot, axis=0)

            # Then, find the centroid of the MDS points
            embedding_centroid = np.mean(embedding, axis=0)

            # Find the translation vector
            translation = centroid - embedding_centroid

            # Translate the MDS points
            embedding = embedding + translation

        # Reset the axes
        ax.clear()

        # Set the axes labels and title (customize as needed)
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_title('MDS Scatter Plot')

        # Set the axes limits (customize as needed)
        ax.set_xlim(-200, 200)
        ax.set_ylim(-200, 200)

        # Put grid on the plot
        ax.grid(color='grey', linestyle='-', linewidth=0.1)

        # Update the scatter plot data
        plt.scatter(embedding[:, 0], embedding[:, 1], c='r')
        plt.scatter(embedding[0, 0], embedding[0, 1], c='b')
        plt.scatter(*np.mean(embedding, axis=0), c='g')

        # Redraw the canvas
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()

        # Update the display
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        screen.blit(surf, (0, 0))
        pygame.display.flip()

        return embedding

    return None


def callback(data):
    global distance_matrix, modified

    if distance_matrix is None:
        raise ValueError("Distance matrix and distance table should be created at this point")

    robot_idx = data.robot_id - ord('A')  # starts at B because A (all) is the broadcast address

    for robot in data.ranges:
        other_robot_idx = robot.other_robot_id - ord('A')
        distance = robot.distance
        # certainty = robot.certainty

        x = robot_idx
        y = other_robot_idx
        if robot_idx > other_robot_idx:
            x = other_robot_idx
            y = robot_idx

        distance_matrix[x, y] = distance

    modified = True


def self_callback(data, args):
    global distance_matrix, distance_table

    if distance_matrix is None or distance_table is None:
        raise ValueError("Distance matrix and distance table should be created at this point")

    self_idx = args[0] - ord('A')

    robot_idx = data.other_robot_id - ord('A')
    distance = data.distance
    # certainty = data.certainty

    distance_table[robot_idx] = distance

    x = self_idx
    y = robot_idx
    if self_idx > robot_idx:
        x = robot_idx
        y = self_idx

    distance_matrix[x, y] = distance


def create_matrix(n: int):
    global distance_matrix, distance_table

    # Create distance matrix
    mask = np.triu_indices(n, k=1)
    matrix = np.zeros((n, n))

    matrix[mask] = 1
    distance_matrix = matrix

    # Create distance table
    distance_table = np.ones((n,))

    if distance_matrix is None or distance_table is None:
        raise ValueError("Couldn't create distance matrix and/or distance table")


def listener():
    global distance_matrix, distance_table, modified

    ros_launch_param = sys.argv[1]

    self_id = ord(sys.argv[1][2])
    n_robots = int(sys.argv[2])

    create_matrix(n_robots)

    if distance_matrix is None or distance_table is None:
        raise ValueError("Distance table should exist at this point, ensure that you called create_matrix() beforehand")

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(f'/{ros_launch_param}/distances', Distances, callback)
    rospy.Subscriber(f'/{ros_launch_param}/distance', Distance, self_callback, (self_id,))
    pub = rospy.Publisher(f'/{ros_launch_param}/direction', Angle, queue_size=10)

    data = []

    counter = 0
    invert_direction = False

    # Save previous values
    previous_plot = None
    current_plot = update_plot(distance_matrix, previous_plot)
    previous_table = np.copy(distance_table)

    with open("output/output.txt", "wb") as f:
        crashed = False
        while not crashed:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or rospy.is_shutdown():
                    crashed = True

            if modified:  # Only render and send message if data has changed
                # TODO: modification should only take place after noise mitigation processes.
                #       Here, it only indicates that new data was received from another agent.

                # Update the data in the plot
                current_plot = update_plot(distance_matrix, previous_plot)
                modified = False

            # Save the data for later stats
            if current_plot is not None:
                data.append(current_plot)

            # Generate morphogenesis enabling message
            msg = generate_morph_msg(self_id - ord('A'), current_plot, previous_table, distance_table)

            # Save current embedding found by MDS to compare with next iteration
            previous_table = np.copy(distance_table)
            previous_plot = np.copy(current_plot)

            # Send the message
            if msg is not None:
                if invert_direction:
                    msg.direction = not msg.direction

                pub.publish(msg)

            previous_msg = msg
            clock.tick(60)  # Limit to 30 frames per second
            counter += 1

        pickle.dump(data, f)


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
