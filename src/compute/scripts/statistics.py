#!/usr/bin/python3
import dataclasses
import datetime

import numpy as np
import rospy
import sys

from tri_msgs.msg import Statistics, Odometry


iterate = False


@dataclasses.dataclass
class Position:
    id: int
    x: float
    y: float
    timestamp: datetime.datetime


def positions_callback(positions, args):
    timestamp = datetime.datetime.utcfromtimestamp(positions.header.stamp.to_sec())
    for position in positions.odometry_data:
        args[0].append(Position(position.id, position.x, position.y, timestamp))

    if args[1] == "agent":
        global iterate
        iterate = True



def listener():
    global iterate

    number_agents = sys.argv[1]
    simulation_positions, estimated_positions = [], []

    agent_id = sys.argv[2]
    print(f'Agent ID: {agent_id}')

    # Listen to statistics ROS topic
    rospy.Subscriber('/simulation/positions', Statistics, positions_callback, args=(simulation_positions, "simulation"))
    rospy.Subscriber(f'/{agent_id}/distance', Statistics, positions_callback, args=(estimated_positions, "agent"))

    while True:
        # Compare real positions to estimated positions of the agent

        # 1. Recover positions of both the agent

        if iterate:
            print(f"Simulation positions: {simulation_positions}")
            print(f"Estimated positions: {estimated_positions}")
            iterate = False


        pass


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass