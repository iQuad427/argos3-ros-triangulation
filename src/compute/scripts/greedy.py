
import numpy as np

from utils import find_rotation_matrix, rotate_and_translate


def algorithm(points, matrix, certainty, uncertainty):
    """

    :param points:
    :param matrix:
    :param certainty: certainty matrix
    :param uncertainty:
    :return:
    """
    if points is None:
        raise ValueError("Points should be set at this point in the loop")

    # Number of points to place
    n = matrix.shape[0]

    # Find the two points with the biggest certainty in the certainty matrix
    chosen_points = list(np.unravel_index(np.argmax(certainty), certainty.shape))

    # Generate the first triangle with a heuristic (the first two points are the base)
    base_positions = np.zeros((2, 2))
    base_positions[0] = np.array([0, 0])
    base_positions[1] = np.array([matrix[chosen_points[0], chosen_points[1]], 0])

    # Rotate the base positions to the points
    rotation_matrix = find_rotation_matrix(points[chosen_points].T, base_positions.T)
    base_positions = base_positions @ rotation_matrix

    # Translate the base positions to the points
    centroid = np.mean(points[chosen_points], axis=0)
    base_centroid = np.mean(base_positions, axis=0)

    translation = centroid - base_centroid
    base_positions = base_positions + translation

    # Translate the first point of the base to the first point of the points
    # translation = points[chosen_points[0]] - base_positions[0]
    # base_positions = base_positions + translation

    # Randomly chose a point not in the chosen points
    not_chosen = [i for i in range(n) if i not in chosen_points]

    # Store estimated positions of the new points
    estimated_positions = np.zeros((n, 2))

    # estimated_positions[chosen_points] = mds_coors_2
    estimated_positions[chosen_points] = base_positions

    # Loop on shuffled remaining points
    position = np.array([0, 0])
    precision = 10
    errors = {}
    for i in not_chosen:
        min_error = np.inf
        for x in np.linspace(points[i, 0] - uncertainty, points[i, 0] + uncertainty, precision):
            for y in np.linspace(points[i, 1] - uncertainty, points[i, 1] + uncertainty, precision):
                new_point = np.array([x, y])

                # Trusted points are the chosen points with bounded uncertainty
                # TODO: Find a better way to chose trusted points (associate to measurements uncertainty)
                trusted_points = [j for j in chosen_points if certainty[i, j] >= 50]

                new_distances = np.linalg.norm(new_point - estimated_positions[trusted_points], axis=1)

                err = np.sum(np.abs(new_distances - matrix[trusted_points, i]))

                errors[(x, y)] = err

                if err < min_error:
                    position = new_point
                    min_error = err

        estimated_positions[i] = position
        chosen_points.append(i)

    rotate_and_translate(points, estimated_positions)

    return estimated_positions
