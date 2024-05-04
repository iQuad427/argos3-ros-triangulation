#!/usr/bin/python3
import signal
import time
import rospy
import sys

from simulation_utils.msg import Manage


compute = True


def signal_handler(sig, frame):
    global compute
    compute = False
    print("Exiting...")


def manager():
    experiment_duration = sys.argv[1]
    experiment_start = sys.argv[2]

    rospy.init_node('simulation_manager', anonymous=True)
    publisher = rospy.Publisher('simulation/manage_command', Manage, queue_size=10)

    time.sleep(30)

    publisher.publish(Manage(stop=False))

    # start = time.time()
    #
    # while time.time() - start < int(experiment_start) and compute:
    #     time.sleep(0.1)
    #
    # print("Starting...")
    #
    # publisher.publish(Manage(stop=False))
    #
    # while time.time() - start < int(experiment_duration) and compute:
    #     time.sleep(0.1)
    #
    # print("Stopping...")
    #
    # publisher.publish(Manage(stop=True))


if __name__ == '__main__':
    # Set signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Enter a function for ROS loop
    try:
        manager()
    except rospy.ROSInterruptException:
        pass
