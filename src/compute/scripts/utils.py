import math

import numpy as np
from numpy.linalg import svd
from sklearn.manifold import MDS
import scipy.stats
from matplotlib import pyplot as plt
from numpy.linalg import norm
from numpy.random import randn, uniform
from tqdm import tqdm

from direction import find_direction_vector_from_position_history, generate_weighted_vector


def MDS_fitting(matrix):
    mds = MDS(n_components=2, dissimilarity='precomputed', normalized_stress=False, metric=True)
    mds_coors = mds.fit_transform(matrix)

    return mds_coors


def remove_uncertain_measurement(distances, certainty, threshold=0.5):
    """
    Remove uncertain measurements from the matrix of distances.

    :param distances: Matrix of distances.
    :param certainty: Certainty of each distance.
    :param threshold: Threshold below which a measurement is considered uncertain.
    :return: Matrix of distances without uncertain measurements. (replaced with -1)
    """
    # Create a copy of the matrix of distances
    distances = distances.copy()

    # Iterate over the matrix of distances
    for i in range(distances.shape[0]):
        for j in range(i, distances.shape[1]):
            # If the certainty is below the threshold, replace the distance with -1
            if i != j and certainty[i, j] < threshold:
                distances[i, j] = -1

    return distances


def remove_uncertain_agents(distances, certainty, threshold=0.5):
    """
    Remove agents with uncertain measurements from the matrix of distances.

    :param distances: Matrix of distances.
    :param certainty: Certainty of each distance.
    :param threshold: Threshold below which a measurement is considered uncertain.
    :return: Matrix of distances without uncertain agents.
    """
    # Create a copy of the matrix of distances
    distances_without_uncertainty = remove_uncertain_measurement(distances, certainty, threshold)

    to_remove = []

    # Iterate over the matrix of distances
    for i in range(distances_without_uncertainty.shape[0]):
        # If any distances for an agent are uncertain, replace the distances with -1
        if np.any(distances_without_uncertainty[:, i] == -1):
            to_remove.append(i)

    # Remove the agents with uncertain measurements
    distances_without_uncertainty = np.delete(distances_without_uncertainty, to_remove, axis=0)
    distances_without_uncertainty = np.delete(distances_without_uncertainty, to_remove, axis=1)

    return distances_without_uncertainty, to_remove


def find_rotation_matrix(X, Y, flipped=False):
    """
    Find the rotation matrix between two point clouds using SVD.

    Parameters:
    - X: Numpy array representing the first point cloud (3xN matrix).
    - Y: Numpy array representing the second point cloud (3xN matrix).
    - flipped : authorize flipping Z plane to better fit the reference point

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

    # special reflection case
    if np.linalg.det(R) < 0 and flipped:
        Vt[:, 1] *= -1
        R = Vt.T @ U.T

    t = -R @ center_X + center_Y

    return R, t


def rotate_and_translate(reference, points, flipped=False):
    rotation, _ = find_rotation_matrix(reference.T, points.T, flipped=flipped)

    # Apply the rotation
    points_rotated = points @ rotation

    # First, find the centroid of the original points
    centroid = np.mean(reference, axis=0)

    # Then, find the centroid of the MDS points
    points_centroid = np.mean(points_rotated, axis=0)

    # Find the translation vector
    translation = centroid - points_centroid

    # Translate the MDS points
    return points_rotated + translation.T


def create_uniform_particles(x_range, y_range, hdg_range, n):
    particles = np.empty((n, 3))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=n)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=n)
    particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=n)
    particles[:, 2] %= 2 * np.pi
    return particles


def create_gaussian_particles(mean, std, n):
    particles = np.empty((n, 3))
    particles[:, 0] = mean[0] + (randn(n) * std[0])
    particles[:, 1] = mean[1] + (randn(n) * std[1])
    particles[:, 2] = mean[2] + (randn(n) * std[2])
    particles[:, 2] %= 2 * np.pi
    return particles


class ParticleFilter:
    def __init__(self, n, initial_pose=None):
        self.N = n

        if initial_pose is not None:
            self.particles = create_gaussian_particles(mean=initial_pose, std=(10, 10, np.pi / 2), n=n)
        else:
            self.particles = create_uniform_particles((-100, 100), (-100, 100), (0, 6.28), n)

        self.weights = np.ones(n) / n

    def predict(self, u, std, dt=1.):
        # Predict a given distance walked by the agent within a time dt
        dist = u[1] * dt + (randn(self.N) * std[1])

        # Choose a direction randomly (if the agent is moving, angle is not changing)
        angle = u[0] + (randn(self.N) * std[0])

        self.particles[:, 0] += np.cos(angle) * dist
        self.particles[:, 1] += np.sin(angle) * dist
        self.particles[:, 2] += angle

    def update(self, z, R, agents_pose_est):
        for i, agent_pose_est in enumerate(agents_pose_est):
            distance = np.linalg.norm(self.particles[:, 0:2] - agent_pose_est[0:2], axis=1)
            self.weights *= scipy.stats.norm(distance, R).pdf(z[i])

        self.weights += 1.e-300
        self.weights /= sum(self.weights)

    def neff(self):
        return 1. / np.sum(np.square(self.weights))

    def resample(self):
        indexes = np.random.choice(range(self.N), size=self.N, p=self.weights)
        self.particles[:] = self.particles[indexes]
        self.weights[:] = self.weights[indexes]
        self.weights /= np.sum(self.weights)

    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0)

    def get_particles(self):
        return self.particles

    def get_weights(self):
        return self.weights


class MultiAgentParticleFilter:
    def __init__(self, a, n, initial_poses=None):
        self.N = n
        if initial_poses is None:
            self.filters = [ParticleFilter(n) for _ in range(a)]
        else:
            self.filters = [ParticleFilter(n, initial_pose) for initial_pose in initial_poses]

    def predict(self, u, std, dt=1.):
        for f in self.filters:
            f.predict(u, std, dt)

    def update(self, z, R, agents_pose_est):
        for f, m in zip(self.filters, z):
            f.update(m, R, agents_pose_est)

    def neff(self):
        return np.mean([f.neff() for f in self.filters])

    def resample(self):
        for f in self.filters:
            f.resample()

    def estimate(self):
        return np.array([f.estimate() for f in self.filters])

    def get_particles(self):
        return [f.get_particles() for f in self.filters]

    def get_weights(self):
        return [f.get_weights() for f in self.filters]


def generate_spiral(num_points, num_turns, branch_spacing, rotation):
    theta = np.linspace(0, num_turns * 2 * np.pi, num_points)
    r = branch_spacing * theta / (2 * np.pi)
    x = r * np.cos(theta + rotation)
    y = r * np.sin(theta + rotation)
    return x, y


def run_pf(n, iters=20, sensor_std_err=10):
    # For 4 agents
    plt.figure()

    multi_agent_pf = MultiAgentParticleFilter(4, n)
    speed = 30

    num_points = iters
    num_turns = 1
    branch_spacing = 100
    rotations = [0, np.pi / 2, np.pi, 3 * np.pi / 2]  # Rotations for each spiral

    # Create the next pauses of the agents as spirals rotated by 90 degrees
    agents_pose = {
        i: generate_spiral(num_points, num_turns, branch_spacing, rotation)
        for i, rotation in enumerate(rotations)
    }

    steps_particles = []
    steps_estimated_pose = []

    for step in tqdm(range(iters)):
        # Plot agents true pose
        for i in range(4):
            plt.scatter(agents_pose[i][0][step], agents_pose[i][1][step], label=f"Agent {i}", color='r', marker='+')

        agents_true_pose = np.array([
            [agents_pose[i][0][step], agents_pose[i][1][step], 0]
            for i in range(4)
        ])

        # Predict
        multi_agent_pf.predict(u=(0, speed), std=(6.28, 5))

        # Z is the distance matrix of real agents
        z = np.zeros((4, 4))
        for j, agent in enumerate(agents_true_pose):
            z[j] = np.linalg.norm(agents_true_pose[:, 0:2] - agent[0:2], axis=1) + (randn(4) * sensor_std_err)

        # Update
        multi_agent_pf.update(z, sensor_std_err, multi_agent_pf.estimate())

        # Resample
        if multi_agent_pf.neff() < n / 2:
            multi_agent_pf.resample()

        # Plot particles
        particles = multi_agent_pf.get_particles()
        colors = ['g', 'b', 'y', 'c']
        # for j, p in enumerate(particles):
        #     plt.scatter(p[:, 0], p[:, 1], color=colors[j], marker=',', s=1)

        steps_particles.append(particles)

        # Plot estimated pose
        estimated_pose = multi_agent_pf.estimate()
        # plt.scatter(estimated_pose[:, 0], estimated_pose[:, 1], color='b', marker='o')

        steps_estimated_pose.append(estimated_pose)

        # Move agents
        # agents_true_pose[:, 0] += np.cos(agents_true_pose[:, 2]) * speed
        # agents_true_pose[:, 1] += np.sin(agents_true_pose[:, 2]) * speed

    # Plot particles
    # steps_particles = np.array(steps_particles)
    colors = ['g', 'b', 'y', 'c']
    # for j in range(4):
    #     plt.scatter(steps_particles[:, j, :, 0], steps_particles[:, j, :, 1], color=colors[j], marker=',', s=1)

    # Plot estimated pose with increasing alpha
    steps_estimated_pose = np.array(steps_estimated_pose)
    for i in range(4):
        for j in range(iters):
            plt.scatter(steps_estimated_pose[j, i, 0], steps_estimated_pose[j, i, 1], color=colors[i], marker='o',
                        alpha=(j + 1) / iters)

    # Plot last estimated positions with index
    for j, p in enumerate(steps_estimated_pose[-1]):
        plt.text(p[0], p[1], str(j), fontsize=12)

    # Plot last true positions with index
    for j in range(4):
        plt.text(agents_pose[j][0][-1], agents_pose[j][1][-1], str(j), fontsize=12)

    plt.show()


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians


distance_direction_history = []
historic_direction_history = []
particle_direction_history = []


def add_direction(distance_direction, historic_direction, particle_direction):
    global distance_direction_history, historic_direction_history, particle_direction_history

    if distance_direction is not None:
        distance_direction_history.insert(0, distance_direction)
        distance_direction_history = distance_direction_history[:3]
    if historic_direction is not None:
        historic_direction_history.insert(0, historic_direction)
        historic_direction_history = historic_direction_history[:3]
    if particle_direction is not None:
        particle_direction_history.insert(0, particle_direction)
        particle_direction_history = particle_direction_history[:3]

    return distance_direction_history, historic_direction_history, particle_direction_history


def compute_angle_from_distances(plot, ref_distances, distances):
    """
    Generate a Direction message for the agent controller to head toward the right direction.

    :param agent: index of the considered agent in the distance table
    :param plot: the plot to take as reference for angles between agents
    :param ref_distances: previous known distances to reference agent
    :param distances: current distances to reference agent

    :return: a Direction ROS message for the robot controller to adapt the direction of the robot
    """
    agent_vector = np.array([plot[0, 0], plot[0, 1]])
    total_vector = np.array([0, 0])
    for i in range(len(distances)):
        direction = (agent_vector - np.array([plot[i, 0], plot[i, 1]]))
        total_vector = total_vector - direction * (ref_distances[i] - distances[i])

    return total_vector


def compute_direction(embedding_hist, positions_hist, distances_hist):
    embedding_hist = [np.array(embedding) for embedding in embedding_hist]
    positions_hist = [np.array(positions) for positions in positions_hist]
    distances_hist = [np.array(distances) for distances in distances_hist]

    # 1. From relative distance evolution
    distance_estimation = np.array([0, 0])
    if len(embedding_hist) > 1 and len(distances_hist) > 1:
        # Compute the average direction from the differences
        direction_vector = compute_angle_from_distances(embedding_hist[-2], distances_hist[-2], distances_hist[-1])
        distance_estimation = direction_vector

    # 2. From historic of estimated positions
    historic_estimation = find_direction_vector_from_position_history(positions_hist)

    # 3. From the particle filter estimated positions and angles
    particle_estimation = np.array([0, 0])
    if all([position.shape[0] == 3 for position in positions_hist]):
        # Compute the average direction from angles in position history with decreasing weight
        angles = np.array([position[2] for position in positions_hist])
        particle_estimation = np.mean(np.array([np.cos(angles), np.sin(angles)]), axis=1)

    # Normalize each vector if non zero norm
    if norm(distance_estimation) != 0:
        distance_estimation /= norm(distance_estimation)
    if norm(historic_estimation) != 0:
        historic_estimation /= norm(historic_estimation)
    if norm(particle_estimation) != 0:
        particle_estimation /= norm(particle_estimation)

    # print(distance_estimation)
    # print(historic_estimation)
    # print(particle_estimation)

    dist_hist, hist_hist, part_hist = add_direction(
        distance_estimation,
        historic_estimation,
        particle_estimation
    )

    return (
        generate_weighted_vector(dist_hist),
        generate_weighted_vector(hist_hist),
        generate_weighted_vector(part_hist)
    )
