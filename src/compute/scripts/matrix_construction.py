#!/usr/bin/python3
import math
import pickle
import sys

import numpy as np
import rospy
import matplotlib.pyplot as plt

from morpho_msgs.msg import Direction
from utils import compute_direction, find_rotation_matrix
from tri_msgs.msg import Distances

from numpy.linalg import svd

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

pygame.display.set_caption("Matplotlib with Pygame")

window = pygame.display.set_mode((600, 600), DOUBLEBUF)
screen = pygame.display.get_surface()

clock = pygame.time.Clock()

previous_plot = None
distance_matrix = None


def update_plot():
    global distance_matrix, previous_plot

    if distance_matrix is not None:
        matrix = distance_matrix

        # Update the data in the plot
        # Make sure the matrix is symmetric
        matrix = (matrix + matrix.T)  # removed '/2' because triangular matrix

        # Use sklearn MDS to reduce the dimensionality of the matrix
        mds = MDS(n_components=2, dissimilarity='precomputed', normalized_stress=False, metric=True, random_state=42)
        embedding = mds.fit_transform(matrix)

        # Rotate dots to match previous plot
        if previous_plot is not None:
            rotation = find_rotation_matrix(previous_plot.T, embedding.T)

            # Apply the rotation
            embedding = embedding @ rotation

            # First, find the centroid of the original points
            centroid = np.mean(previous_plot, axis=0)

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


def MDS_fitting(matrix):
    mds = MDS(n_components=2, dissimilarity='precomputed', normalized_stress=False, metric=True)
    mds_coors = mds.fit_transform(matrix)

    return mds_coors


def callback(data):
    global distance_matrix

    robot_idx = data.robot_id - ord('B')  # starts at B because A (all) is the broadcast address

    for robot in data.ranges:
        other_robot_idx = robot.other_robot_id - ord('B')
        distance = robot.distance

        x = robot_idx
        y = other_robot_idx
        if robot_idx > other_robot_idx:
            x = other_robot_idx
            y = robot_idx

        distance_matrix[x, y] = distance


def create_matrix(n: int):
    global distance_matrix

    mask = np.triu_indices(n, k=1)
    matrix = np.zeros((n, n))

    matrix[mask] = 1
    distance_matrix = matrix


def listener():
    global previous_plot

    ros_launch_param = sys.argv[1]
    n_robots = int(sys.argv[2])

    create_matrix(n_robots)

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(f'/{ros_launch_param}/distances', Distances, callback)
    pub = rospy.Publisher(f'/{ros_launch_param}/direction', Direction, queue_size=10)

    data = []
    max_gradient = 0.1
    previous_gradient = 0

    with open("output.txt", "wb") as f:
        crashed = False
        while not crashed:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or rospy.is_shutdown():
                    crashed = True

            # Update the data in the plot
            current_plot = update_plot()
            msg = compute_direction(previous_plot, current_plot)

            previous_plot = current_plot

            if msg is not None:
                msg.gradient = (msg.gradient + previous_gradient)/2

                if abs(msg.gradient) > max_gradient:
                    max_gradient = abs(msg.gradient)

                msg.gradient = msg.gradient/max_gradient
                msg.activation = 1 / (1 + math.exp(-msg.gradient))

                print(msg)
                pub.publish(msg)

            # Update the data in the plot
            embedding = update_plot()
            if embedding is not None:
                data.append(embedding)
                # print(embedding)

            clock.tick(30)  # Limit to 30 frames per second

        pickle.dump(data, f)


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
