import math
import numpy as np

from morpho_msgs.msg import Direction, Angle


class Vector:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, other):
        if not isinstance(other, Vector):
            raise ValueError("Vectorial Addition is the only supported addition")

        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        if not isinstance(other, Vector):
            raise ValueError("Vectorial Subtraction is the only supported subtraction")

        return self.__add__(-other)

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Vector(self.x * other, self.y * other)
        elif isinstance(other, Vector):
            return self.x * other.x + self.y * other.y
        else:
            raise ValueError("Scalar Multiplication and Scalar Product are the only supported multiplication")

    def __truediv__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Vector(self.x / other, self.y / other)
        else:
            raise ValueError("Division by a Scalar is the only supported division")

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __str__(self):
        return f"({self.x}, {self.y})"

    def to_numpy(self):
        return np.array([self.x, self.y])

    def norm(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

    def normalize(self):
        factor = self.norm()
        self.x = self.x / factor
        self.y = self.y / factor

        return self

    def rotate(self, angle=90):
        rotation = angle * math.pi / 180
        x = self.x*math.cos(rotation) - self.y*math.sin(rotation)
        y = self.x*math.sin(rotation) + self.y*math.cos(rotation)
        return Vector(x, y)


def compute_direction(previous_plot, current_plot):
    """
    Compute direction to centroid of the swarm using evolution of shape estimation

    :param previous_plot: previous positions of the swarm
    :param current_plot: current positions of the swarm
    :return: a Direction ROS message describing the evolution of distance and the angle to the centroid of the swarm
    """

    # Note: The pairwise evolution of distances between the agents (with the matrix of distances) may be computed
    #       as a difference of the matrices to estimate the gradient and angle of the movement of the agent.

    if previous_plot is not None and current_plot is not None:
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


def generate_morph_msg(agent, plot, ref_distances, distances):
    """
    Generate a Direction message for the agent controller to head toward the right direction.

    :param agent: index of the considered agent in the distance table
    :param plot: the plot to take as reference for angles between agents
    :param ref_distances: previous known distances to reference agent
    :param distances: current distances to reference agent

    :return: a Direction ROS message for the robot controller to adapt the direction of the robot
    """
    agent_vector = np.array([plot[agent, 0], plot[agent, 1]])
    total_vector = np.array([0, 0])
    for i in range(len(distances)):
        if i != agent:
            direction = (agent_vector - np.array([plot[i, 0], plot[i, 1]]))
            total_vector = total_vector + direction * (ref_distances[i] - distances[i])

    centroid = np.mean(plot, axis=0)  # Should always be (0, 0) due to plot update shakiness mitigation
    target = np.array([centroid[0], centroid[1]])

    target_vector = (target - agent_vector)
    normal_vector = np.array([
        target_vector[0] * math.cos(math.pi / 2) - target_vector[1] * math.sin(math.pi / 2),
        target_vector[0] * math.sin(math.pi / 2) + target_vector[1] * math.cos(math.pi / 2)
    ])

    theta = np.arccos(np.dot(-target_vector, total_vector) / (np.linalg.norm(target_vector) * np.linalg.norm(total_vector)))

    # Find direction of rotation (compared to plot: True if trigonometric, False if counter-trigonometric)
    direction = np.dot(total_vector, normal_vector) > 0

    return Angle(distance=np.linalg.norm(target_vector), angle=theta, direction=direction)


def find_direction_vector_from_position_history(position_history):
    """
    Given a position history, find the direction vector of the movement
    :param position_history: A list of positions
    :return: The direction vector
    """
    # Convert the list to a numpy array
    position_history = np.array(position_history)

    # Compute the difference between each position
    diff = position_history[1:] - position_history[:-1]

    # Compute the direction vector by taking the mean of the differences
    direction_vector = np.mean(diff, axis=0)

    # Normalize the direction vector

    direction_vector = direction_vector / np.linalg.norm(direction_vector)

    return direction_vector


def generate_weighted_vector(vectors):
    """
    Generate a new vector by giving more weight to the most recent vectors.

    Args:
    vectors (list of tuples): List of 2D vectors represented as tuples.

    Returns:
    tuple: The weighted average vector.
    """
    # Define weights for each vector
    weights = [0.9 ** i for i in range(len(vectors))]

    # Calculate weighted sum for x and y components separately
    weighted_x_sum = sum(weight * vector[0] for weight, vector in zip(weights, vectors))
    weighted_y_sum = sum(weight * vector[1] for weight, vector in zip(weights, vectors))

    # Calculate total weight
    total_weight = sum(weights)

    # Calculate weighted average
    weighted_average_x = weighted_x_sum / total_weight
    weighted_average_y = weighted_y_sum / total_weight

    return weighted_average_x, weighted_average_y


def range_and_bearing(agent, direction, distances, historic, plot):
    """
    Estimate the relative direction of all agents

    :param agent: id of the robot currently estimating its range and bearing to other robots of the swarm
    :param direction: direction vector of the agent
    :param historic: sample of last known position of the robot on the plot
    :param plot: position estimation to take into consideration when computing angles
    :return: the estimated readings of the simulated range and bearing sensor
    """
    # Convert the plot to a numpy array
    plot = np.array(plot)
    historic = np.array(historic)

    # Rotate the plot to have the agent of interest at the origin and the direction vector aligned with the x-axis

    # Compute the direction vector of the movement
    direction_vector = direction

    # Compute the angle between the direction vector and the x-axis
    angle = np.arctan2(direction_vector[1], direction_vector[0])

    # Compute the rotation matrix
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    # Rotate the plot
    plot = np.array(plot) - historic[-1]
    plot = plot @ rotation_matrix

    # Compute the range and bearing to the other agents
    _rab = []

    for x, position in enumerate(plot):
        if x != agent:
            # Compute the relative position of the other agent
            relative_position = position - plot[agent]

            # Compute the range and bearing
            _rab.append(
                [
                    distances[agent, x],
                    np.arctan2(relative_position[1], relative_position[0])
                ]
            )

    return _rab
