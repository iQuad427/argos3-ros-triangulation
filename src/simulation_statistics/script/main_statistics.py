#!/usr/bin/python3
import dataclasses
import datetime
import signal
import sys
from typing import List

import rospy

from simulation_utils.msg import Distances, Distance, Odometry, Positions, Manage


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


@dataclasses.dataclass
class Position:
    id: int

    x: float
    y: float
    z: float

    a: float
    b: float
    c: float
    d: float

    def __repr__(self):
        return f"{self.id},{self.x},{self.y}"


@dataclasses.dataclass
class Memory:
    positions: List[Position]
    timestamp: float

    def __repr__(self):
        buffer = f"{self.timestamp}&"
        for position in self.positions:
            buffer += f"{position}#"

        return buffer[:-1]


iterate = False
stop = False


def callback(msg):
    global stop

    if msg.stop:
        stop = True


# Add handler for SIGINT
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



def positions_callback(positions, args):
    timestamp = positions.timestep / 10

    storage = args[0]

    memory = []
    for position in positions.odometry_data:
        memory.append(
            Position(
                position.id if position.id < ord('A') else position.id - ord('A'),
                position.x if args[1] == "agent" else position.x * 100,
                position.y if args[1] == "agent" else position.y * 100,
            )
        )

    storage.append(Memory(memory, timestamp))

    if args[1] == "agent":
        global iterate
        iterate = True

    # Keep only the first and last one (use reference to list)
    storage = storage[:0] + storage[-1:]


def listener():
    global iterate

    output_dir = sys.argv[1]
    output_file = sys.argv[2]

    number_agents = sys.argv[3]
    estimated_positions = []
    simulation_positions = []

    # TODO: Generate one memory for each list
    # estimated_positions.append(
    #     Memory()
    # )

    agent_id = sys.argv[4]
    print(f'Agent ID: {agent_id}')

    rospy.init_node('statistics_computation', anonymous=True)

    # Subscribe to the manager command (to stop the node when needed)
    rospy.Subscriber('simulation/manage_command', Manage, callback)

    # Listen to statistics ROS topic
    rospy.Subscriber(f'{agent_id}/positions', Positions, positions_callback, (estimated_positions, "agent"))
    rospy.Subscriber(f'simulation/positions', Positions, positions_callback, (simulation_positions, "simulation"))

    # Name of file is statistics_output_dd_mm_hh_mm.txt
    with open(f"{output_dir}/{output_file}", "w+") as f:
        while not stop:
            if iterate:
                ground_truth = simulation_positions[-1]
                estimation = estimated_positions[-1]

                start = estimated_positions[0].timestamp
                now = (estimation.timestamp - start)

                f.write(f"estim={now:0.2f}={estimation}\n")
                f.write(f"truth={now:0.2f}={ground_truth}\n")

                iterate = False


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
