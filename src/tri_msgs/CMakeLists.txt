cmake_minimum_required(VERSION 3.0.2)
project(tri_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
        DIRECTORY msg
        FILES
        Odometry.msg
        Statistics.msg
        Distance.msg
        Distances.msg
        Item.msg
        Matrix.msg
        Agent.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
