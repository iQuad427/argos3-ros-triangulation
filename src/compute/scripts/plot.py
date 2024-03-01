
import numpy as np
import pickle
import matplotlib.pyplot as plt
from numpy.linalg import svd

from src.compute.scripts.utils import find_rotation_matrix

if __name__ == '__main__':
    # Read the output file, and plot the error over time
    # Error is the mean square error between the expected position and the estimated position

    # Expected position (4 points on the corner of square of 100 of side)
    expected = np.array([[0, 0], [0, 100], [100, 0], [100, 100]])

    with open("output.txt", "rb") as f:
        error = []

        data = pickle.load(f)

        for embedding in data:

            print(embedding)

            # Rotate and translate the estimated position to fit the expected position
            rotation = find_rotation_matrix(expected.T, embedding.T)

            # Apply the rotation
            embedding = embedding @ rotation

            # First, find the centroid of the original points
            centroid = np.mean(expected, axis=0)

            # Then, find the centroid of the MDS points
            embedding_centroid = np.mean(embedding, axis=0)

            # Find the translation vector
            translation = centroid - embedding_centroid

            # Translate the MDS points
            embedding = embedding + translation

            # Compute the mean square error between the expected position and the estimated position
            mse = np.mean((expected - embedding)**2)

            if mse < 10_000:
                error.append(mse)

        print(len(error))

        # Plot the error over time
        plt.plot(np.arange(len(error)) / 30, error)
        plt.xlabel('Time (seconds)')
        plt.ylabel('Mean square error')
        plt.title('Error over time')
        plt.show()
