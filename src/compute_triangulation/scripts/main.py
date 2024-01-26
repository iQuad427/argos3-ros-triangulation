#! /usr/bin/env python3
import math
import threading

import numpy as np
import rospy
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from numpy.linalg import svd

from sklearn.manifold import MDS

from tri_msgs.msg import Agent, Matrix, Item
from std_msgs.msg import MultiArrayLayout

import matplotlib
matplotlib.use("Agg")

import matplotlib.backends.backend_agg as agg
import pylab
import pygame
from pygame.locals import *

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


def listen():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/fbA/distance_matrix', Agent, callback)

    crashed = False
    while not crashed:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or rospy.is_shutdown():
                crashed = True

        # Update the data in the plot
        update_plot()

        clock.tick(30)  # Limit to 30 frames per second


def find_rotation_matrix(X, Y):
    """
    Find the rotation matrix between two point clouds using SVD.

    Parameters:
    - X: Numpy array representing the first point cloud (3xN matrix).
    - Y: Numpy array representing the second point cloud (3xN matrix).

    Returns:
    - R: 3x3 rotation matrix.
    """

    # Center the point clouds
    center_X = X.mean(axis=1).reshape(-1, 1)
    center_Y = Y.mean(axis=1).reshape(-1, 1)

    X_centered = X - center_X
    Y_centered = Y - center_Y

    # Compute the covariance matrix H
    H = X_centered @ Y_centered.T

    # Perform SVD on H
    U, _, Vt = svd(H)

    # Compute the rotation matrix R
    R = Vt.T @ U.T

    return R


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
        previous_plot = embedding

        # Redraw the canvas
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()

        # Update the display
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        screen.blit(surf, (0, 0))
        pygame.display.flip()


def get_matrix_dim(layout: MultiArrayLayout):
    label = ()
    dim = ()
    stride = ()

    for i, d in enumerate(layout.dim):
        label += (d.label,)
        dim += (d.size,)
        stride += (d.stride,)

    return label, dim, stride


def print_matrix(matrix: Matrix):
    label, dim, stride = get_matrix_dim(matrix.layout)

    for i in range(dim[0]):
        buffer = ""
        for j in range(dim[1]):
            buffer += f"{matrix.data[i * stride[0] + j * stride[1]].distance} "

        print(buffer)


def convert_to_numpy(matrix: Matrix):
    global distance_matrix

    label, dim, stride = get_matrix_dim(matrix.layout)

    dist_mat = np.zeros((dim[0], dim[0]), dtype=float)

    for i in range(dim[0]):
        for j in range(dim[1]):
            dist_mat[i, j] = matrix.data[i * stride[0] + j * stride[1]].distance

            # if dist_mat[i, j] == 0:
            #     dist_mat[i, j] = 0

    return dist_mat


def MDS_fitting(matrix):
    mds = MDS(n_components=2, dissimilarity='precomputed', normalized_stress=False, metric=True)
    mds_coors = mds.fit_transform(matrix)

    return mds_coors


def callback(data: Agent):
    global distance_matrix

    rospy.loginfo(f"received")

    distance_matrix = convert_to_numpy(data.distance_matrix)
    print(distance_matrix)


if __name__ == '__main__':
    # Read robot 0 output and showcase the triangulation results
    print("hello!")
    listen()



