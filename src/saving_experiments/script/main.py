#!/usr/bin/python3
import datetime
import signal
import sys

import rospy
import tqdm
from simulation_utils.msg import Manage, Distance, Distances


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


def parse_message(line: str) -> Distances:
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


def unparse_message(msg: Distances, agent_id) -> str:
    line = f"{chr(msg.robot_id)}=0:100"
    for data in msg.ranges:
        if chr(data.other_robot_id) != chr(msg.robot_id):
            line += f",{chr(data.other_robot_id)}={data.distance}:{data.certainty}"
    return f"{line}"


def make_distances(data: Distance, agent_id) -> Distances:
    msg = Distances()
    msg.ranges = [data]
    msg.robot_id = ord(agent_id)
    return msg


def talker():
    global start, stop

    simulation = True if sys.argv[1] == "True" else False

    # Parse arguments
    agent_id = sys.argv[2]

    output_dir = sys.argv[3]
    output_file = sys.argv[4]

    rospy.init_node('simulation_sensor_measurement', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    rospy.Subscriber('simulation_manage_command', Manage, callback)

    # Publish the read distances
    publisher = rospy.Publisher('simulation_sensor_read', Distances, queue_size=10)

    while not start and not stop:
        pass

    if not simulation:
        print("Started listening to ROS")

        historical_data = []
        rospy.Subscriber(f'/{agent_id}/distances', Distances, lambda data: historical_data.append(data))
        rospy.Subscriber(f'/{agent_id}/distance', Distance, lambda data: historical_data.append(make_distances(data, agent_id[2])))

        start_time = datetime.datetime.now()

        with open(f"{output_dir}/{output_file}", "w+") as f:
            while not rospy.is_shutdown() and start and not stop:
                if len(historical_data) == 0:
                    continue

                msg = historical_data.pop()
                publisher.publish(msg)

                line = unparse_message(msg, agent_id[2])
                print(line)
                f.write(f"{(datetime.datetime.now() - start_time).total_seconds()}&{line}\n")
    else:
        print("Started reading from file")

        start_time = datetime.datetime.now()

        with open(f"{output_dir}/{output_file}", "r") as f:
            # Read file, line by line and output only if timestamp is reached
            lines = f.readlines()

        for line in tqdm.tqdm(lines):
            msg, faulty_frame = parse_message(line.split("&")[1])
            if not faulty_frame:
                # While not the right moment, do nothing
                while (datetime.datetime.now() - start_time).total_seconds() < float(line.split("&")[0]):
                    pass

                publisher.publish(msg)
            if stop:
                break


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
