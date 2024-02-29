import math

import numpy as np
from numpy.linalg import svd
import matplotlib.pyplot as plt
from sklearn.manifold import MDS

from morpho_msgs.msg import Direction


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


def compute_direction(previous_plot, current_plot):
    # TODO: complete gradient and angle measurement
    # Note: The pairwise evolution of distances between the agents (with the matrix of distances) may be computed
    #       as a difference of the matrices to estimate the gradient and angle of the movement of the agent.

    if previous_plot is not None and current_plot is not None:
        # print(previous_plot)
        # print(current_plot)
        # Compute the centroid of the current and previous MDS coordinates
        centroid_previous = np.mean(previous_plot, axis=0)
        centroid_current = np.mean(current_plot, axis=0)

        # Compute distance between agent and centroid
        distance_before = np.linalg.norm(centroid_previous - previous_plot[0])
        distance_after = np.linalg.norm(centroid_current - current_plot[0])

        # Compute the gradient and angle of the movement
        gradient = distance_before - distance_after
        angle = math.atan2(centroid_current[1] - centroid_previous[1], centroid_current[0] - centroid_previous[0])

        msg = Direction()

        msg.distance = distance_after
        msg.gradient = gradient
        msg.angle = angle
    else:
        msg = None

    return msg
