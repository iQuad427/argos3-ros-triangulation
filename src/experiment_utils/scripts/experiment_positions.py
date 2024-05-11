#!/usr/bin/python3

# Read the odometry messages from a list of topics and combine them into a Positions message

import rospy
from nav_msgs.msg import Odometry
from experiment_utils.msg import Positions, Odometry as CustomOdometry


if __name__ == '__main__':
    rospy.init_node('experiment_positions', anonymous=True)

    # Get the list of topics to read from
    topics = [
        "/epuck_17/odometry/filtered"
        "/epuck_37/odometry/filtered"
        "/epuck_38/odometry/filtered"
        "/epuck_40/odometry/filtered"
    ]
    robots = [
        "B",  # Don't know which robot is which
        "C",
        "D",
        "E"   # This one is certain
    ]

    # Create the publisher
    publisher = rospy.Publisher('/experiment/positions', Positions, queue_size=10)

    # Create the message
    dict_positions = {
        f"{topic}": [] for topic in topics
    }

    # Create the callback function
    def callback(data, args):
        # Get the topic name
        topic = args[0]
        robot = args[1]

        # Add message to the dictionary
        dict_positions[topic].append(data)

        # Check if all topics have sent a message
        if all([len(dict_positions[topic]) > 0 for topic in topics]):
            # Create the message
            msg = Positions()

            # Add the positions to the message
            for topic in topics:
                msg.positions.append(
                    CustomOdometry(
                        id=robot,
                        pose=dict_positions[topic][-1].pose
                    )
                )

            # Publish the message
            publisher.publish(msg)

            # Clear the dictionary
            for topic in topics:
                dict_positions[topic] = []

    # Create the subscribers
    subscribers = []
    for i, topic in enumerate(topics):
        subscribers.append(rospy.Subscriber(topic, Odometry, callback, args=(topic, robots[i])))

    # Wait for the messages
    rospy.spin()
