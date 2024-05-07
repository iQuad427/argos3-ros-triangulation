#!/usr/bin/python3
import datetime
import math
import signal
import sys
import time
from collections import defaultdict

import rospy
import tqdm
from simulation_utils.msg import Manage, Distance, Distances, Positions, Odometry


stop = False
start = False


def callback(msg):
    global stop, start

    print("Callback:", msg.stop)

    if msg.stop:
        stop = True
    else:
        start = True


def signal_handler(sig, frame):
    global stop
    stop = True


def parse_distances(line: str) -> Distances:
    msg = Distances()
    msg.ranges = []

    faulty_frame = False

    infos = line.split(",")

    if len(infos) > 1:  # Means that the split was made (therefore at least one information to process)
        sender_robot = infos[0]
        sender_info = sender_robot.split("=")

        if sender_info[1] == "0:100":
            msg.robot_id = ord(sender_info[0])

        for info in infos[1:]:
            try:
                robot, distance = info.split("=")
                distance, certainty = distance.split(":")

                data = Distance()

                data.other_robot_id = ord(robot)
                data.distance = int(distance)
                data.certainty = int(certainty)

                msg.ranges.append(data)
            except ValueError as e:
                pass

    return msg, faulty_frame


def unparse_distances(msg: Distances) -> str:
    line = f"{chr(msg.robot_id)}=0:100"
    for data in msg.ranges:
        if chr(data.other_robot_id) != chr(msg.robot_id):
            line += f",{chr(data.other_robot_id)}={data.distance}:{data.certainty}"
    return f"{line}"


def parse_positions(line: str) -> Positions:
    msg = Positions()
    msg.odometry_data = []

    faulty_frame = False

    infos = line.split("#")

    if len(infos) > 1:
        for info in infos:
            agent_id, positions, orientations = info.split(":")
            x, y, z = [float(i) for i in positions.split(",")]
            a, b, c, d = [float(j) for j in orientations.split(",")]

            msg.odometry_data.append(
                Odometry(
                    id=ord(agent_id),
                    x=x, y=y, z=z,
                    a=a, b=b, c=c, d=d
                )
            )
    else:
        faulty_frame = True

    return msg, faulty_frame


def unparse_positions(msg: Positions) -> str:
    line = f""
    for position in msg.odometry_data:
        if line:
            line += "#"
        # TODO: add the orientation information
        line += f"{chr(position.id)}:{position.x},{position.y},{position.z}:{position.a},{position.b},{position.c},{position.d}"
    return f"{line}"


def add_timestamp(data, start_time):
    data.timestamp = (datetime.datetime.now() - start_time).total_seconds()
    return data


def talker():
    global start, stop

    simulation = True if sys.argv[1] == "True" else False

    # Parse arguments
    agent_id = sys.argv[2]

    output_dir = sys.argv[3]
    output_file = sys.argv[4]

    iteration_rate = int(sys.argv[5])

    # TODO: add iteration per second of the simulation to take into account when computing
    #       timestamps of simulation messages

    rospy.init_node('simulation_streamer', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    rospy.Subscriber('simulation/manage_command', Manage, callback)

    if not simulation:
        print("Started listening to ROS")

        historical_data = defaultdict(list)
        simulation_data = defaultdict(list)

        rospy.Subscriber(f'/{agent_id}/distances', Distances, lambda data: historical_data[data.timestamp].append(data), queue_size=12000)
        rospy.Subscriber(f'/simulation/positions', Positions, lambda data: simulation_data[data.timestamp].append(data), queue_size=12000)

        while not rospy.is_shutdown() and not stop:
            pass

        print("Started writing to file")

        # Start time is smallest timestep in the simulation data
        start_time = int(min(simulation_data.keys()))

        with open(f"{output_dir}/{output_file}", "w+") as f:
            for step in range(start_time, int(max(historical_data.keys()))):
                distances = historical_data[step]
                positions = simulation_data[step]

                for distance in distances:
                    distance_line = unparse_distances(distance)
                    f.write(f"{step/iteration_rate}&distances&{distance_line}\n")

                for position in positions:
                    position_line = unparse_positions(position)
                    f.write(f"{step/iteration_rate}&positions&{position_line}\n")
    else:
        print("Started reading from file")

        # Publish the read distances
        agent_publisher = rospy.Publisher(f'/{agent_id}/distances', Distances, queue_size=10)
        simulation_publisher = rospy.Publisher(f'/simulation/positions', Positions, queue_size=10)

        with open(f"{output_dir}/{output_file}", "r") as f:
            # Read file, line by line and output only if timestamp is reached
            lines = f.readlines()

        time.sleep(3)

        print("START STREAMING")

        start_time = datetime.datetime.now()

        min_step = math.inf
        for line in tqdm.tqdm(lines):
            timestamp, msg_type, data = line.split("&")
            min_step = min(min_step, float(timestamp))
            timestamp = float(timestamp) - min_step

            message = None
            publisher = None
            failed = True

            if msg_type == "distances":
                distances, failed = parse_distances(data)
                if not failed:
                    message = distances
                    publisher = agent_publisher
            elif msg_type == "positions":
                positions, failed = parse_positions(data)
                if not failed:
                    simulation_publisher.publish(positions)

            if not failed and message is not None and publisher is not None:
                # While not the right moment, do nothing
                while (datetime.datetime.now() - start_time).total_seconds() < float(timestamp):
                    pass

                message.timestamp = float(timestamp)
                publisher.publish(message)

            if stop:
                break

        print("STOP STREAMING")


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
