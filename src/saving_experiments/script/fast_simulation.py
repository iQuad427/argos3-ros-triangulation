#!/usr/bin/python3
import datetime
import signal
import sys

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
    rospy.Subscriber('simulation/manage_command', Manage, callback)

    # while not start and not stop:
    #     pass

    if not simulation:
        print("Started listening to ROS")

        historical_data = []
        simulation_data = []

        # TODO: put together distance.s in the topic distances
        rospy.Subscriber(f'/{agent_id}/distances', Distances, lambda data: historical_data.append(data))
        rospy.Subscriber(f'/{agent_id}/distance', Distance, lambda data: historical_data.append(make_distances(data, agent_id[2])))

        rospy.Subscriber(f'/simulation/positions', Positions, lambda data: simulation_data.append(data))

        start_time = datetime.datetime.now()

        with open(f"{output_dir}/{output_file}", "w+") as f:
            while not rospy.is_shutdown() and start and not stop:
                if len(historical_data) == 0 or len(simulation_data) == 0:
                    continue

                # TODO: might want to ensure that all distances are sent in the right order (pop at index 0)
                distances = historical_data.pop()
                positions = simulation_data.pop()

                distances_line = unparse_distances(distances)
                positions_line = unparse_positions(positions)

                timestep = (datetime.datetime.now() - start_time).total_seconds()

                f.write(f"{timestep}&{distances_line}&{positions_line}\n")
    else:
        print("Started reading from file")

        start_time = datetime.datetime.now()

        # Publish the read distances
        agent_publisher = rospy.Publisher(f'/{agent_id}/distances', Distances, queue_size=10000)
        simulation_publisher = rospy.Publisher(f'/simulation/positions', Positions, queue_size=1000)

        with open(f"{output_dir}/{output_file}", "r") as f:
            # Read file, line by line and output only if timestamp is reached
            lines = f.readlines()

        for line in tqdm.tqdm(lines):
            timestep, distances, positions = line.split("&")

            distances_msg, status_1 = parse_distances(distances)
            positions_msg, status_2 = parse_positions(positions)
            if not status_1 and not status_2:
                # While not the right moment, do nothing
                while (datetime.datetime.now() - start_time).total_seconds() < float(timestep):
                    pass

                agent_publisher.publish(distances_msg)
                simulation_publisher.publish(positions_msg)
            if stop:
                break


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
