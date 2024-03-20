import math

import numpy as np
from numpy.linalg import svd
from sklearn.manifold import MDS


def MDS_fitting(matrix):
    mds = MDS(n_components=2, dissimilarity='precomputed', normalized_stress=False, metric=True)
    mds_coors = mds.fit_transform(matrix)

    return mds_coors


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


def rotate_and_translate(reference, points):
    rotation = find_rotation_matrix(reference.T, points.T)

    # Apply the rotation
    points_rotated = points @ rotation

    # First, find the centroid of the original points
    centroid = np.mean(reference, axis=0)

    # Then, find the centroid of the MDS points
    points_centroid = np.mean(points_rotated, axis=0)

    # Find the translation vector
    translation = centroid - points_centroid

    # Translate the MDS points
    return points_rotated + translation
