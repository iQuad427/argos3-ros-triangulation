import os

from experiment_utils.msg import Positions, Odometry as CustomOdometry
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import rosbag


if __name__ == '__main__':
    experiments_directory = "/home/quentin/Dev/argos3-ros-triangulation/src/experiment_utils"

    topics = [
        "/epuck_17/odometry/filtered",
        "/epuck_37/odometry/filtered",
        "/epuck_38/odometry/filtered",
        "/epuck_40/odometry/filtered"
    ]

    robots = [
        "B",  # Don't know which robot is which
        "C",
        "D",
        "E"   # This one is certain
    ]

    # For directories in experiments
    experiments = os.listdir(f"{experiments_directory}/input")
    for experiment in experiments:

        # List files in the directory
        files = os.listdir(f"{experiments_directory}/input/{experiment}")

        # Get the input and output bag (tracking_data_*.bag)
        input_bags = [
            f"{experiments_directory}/input/{experiment}/" + [f for f in files if "tracking_data" in f][0],
            f"{experiments_directory}/input/{experiment}/" + [f for f in files if "sensor_data" in f][0],
            ]
        output_bag = f"{experiments_directory}/output/{experiment}/complete_experiment.bag"

        # Create output directory if it doesn't exist
        if not os.path.exists(f"{experiments_directory}/output/{experiment}"):
            os.makedirs(f"{experiments_directory}/output/{experiment}")

        dict_positions = {
            f"{topic}": [] for topic in topics
        }

        with rosbag.Bag(output_bag, 'w') as out_bag:
            for input_bag in input_bags:
                for topic, msg, t in rosbag.Bag(input_bag).read_messages():
                    if topic in topics:
                        dict_positions[topic].append(msg)

                        if all([len(dict_positions[topic]) > 0 for topic in topics]):
                            msg = Positions()
                            msg.header = Header()
                            msg.header.stamp = t
                            for top in topics:
                                msg.positions.append(
                                    CustomOdometry(
                                        id=ord(robots[topics.index(top)]),
                                        pose=dict_positions[top][-1].pose.pose
                                    )
                                )
                            out_bag.write("/experiment/positions", msg, t)

                            # Clear the dictionary
                            dict_positions = {
                                f"{top}": [] for top in topics
                            }

                        out_bag.write(topic, msg, t)
                    else:
                        out_bag.write(topic, msg, t)
