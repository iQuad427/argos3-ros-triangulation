#include "statistics_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

/****************************************/
/****************************************/

/*
 * To reduce the number of waypoints stored in memory,
 * consider two robot positions distinct if they are
 * at least MIN_DISTANCE away from each other
 * This constant is expressed in meters
 */
static const Real MIN_DISTANCE = 0.05f;
/* Convenience constant to avoid calculating the square root in PostStep() */
static const Real MIN_DISTANCE_SQUARED = MIN_DISTANCE * MIN_DISTANCE;

int CStatisticsLoopFunctions::count;

CStatisticsLoopFunctions::CStatisticsLoopFunctions() :
        m_nRobots(10) {}

/****************************************/
/****************************************/

void CStatisticsLoopFunctions::InitROS() {
    //get e-puck ID
    std::stringstream name;
    name.str("");
    name << "simulation"; // fbX

    //init ROS
    if (!ros::isInitialized()) {
        char **argv = NULL;
        int argc = 0;
        ros::init(argc, argv, name.str());
    }

    //ROS access node
    ros::NodeHandle node;

    std::stringstream distancePublisherName;
    std::stringstream positionPublisherName;

    distancePublisherName << name.str() << "/distance_matrix";
    positionPublisherName << name.str() << "/positions";

    // Register the publisher to the ROS master
    m_matrixPublisher = node.advertise<tri_msgs::Agent>(distancePublisherName.str(), 10);
    m_positionPublisher = node.advertise<tri_msgs::Statistics>(positionPublisherName.str(), 10);

    // Prefill Messages
    m_matrixMessage.header.frame_id = distancePublisherName.str();
    m_matrixMessage.agent_id = (uint8_t) 0;

    for (int i = 0; i < m_nRobots; ++i) {
        m_matrixMessage.translate.push_back('A' + i);
    }

    // Define Matrix Dimensions (constant for now)
    std_msgs::MultiArrayDimension dim_1;
    std_msgs::MultiArrayDimension dim_2;

    dim_1.label = "row";
    dim_1.size = m_nRobots;
    dim_1.stride = m_nRobots;

    dim_2.label = "column";
    dim_2.size = m_nRobots;
    dim_2.stride = 1;

    m_matrixMessage.distance_matrix.layout.dim.push_back(dim_1);
    m_matrixMessage.distance_matrix.layout.dim.push_back(dim_2);
    m_matrixMessage.distance_matrix.layout.data_offset = 0;
}

void CStatisticsLoopFunctions::ControlStepROS() {
    if (ros::ok()) {
        /* Fill in a message and publish using the publisher node */
        m_matrixMessage.header.stamp = ros::Time::now();

        // Add the matrix
        tri_msgs::Item item;

//        // TODO: optimize data update, avoid creating everything from scratch each time
//        m_matrixMessage.distance_matrix.data.clear();
//
//        // Access and print the elements of the matrix
//        for (int i = 0; i < m_nRobots; ++i) {
//            for (int j = 0; j < m_nRobots; ++j) {
//                item.distance = m_distanceMatrix[i][j].first;
//                item.discount = m_distanceMatrix[i][j].second;
//                m_matrixMessage.distance_matrix.data.push_back(item);
//            }
//        }
//
//        m_matrixPublisher.publish(m_matrixMessage);

        /* Get the map of all foot-bots from the space */
        CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
        /* Go through them */

        tri_msgs::Statistics statistics;
        statistics.header.stamp = ros::Time::now();
        statistics.header.frame_id = "simulation";

        for (CSpace::TMapPerType::iterator it = tFBMap.begin(); it != tFBMap.end(); ++it) {
            tri_msgs::Odometry odometry;

            /* Create a pointer to the current foot-bot */
            CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
            CVector3 position = pcFB->GetEmbodiedEntity().GetOriginAnchor().Position;
            CQuaternion orientation = pcFB->GetEmbodiedEntity().GetOriginAnchor().Orientation;

            // Header
            odometry.id = (int8_t) pcFB->GetId()[2];

            // Position
            odometry.x = position[0];
            odometry.y = position[1];
            odometry.z = position[2];

            // Orientation
            odometry.a = orientation.GetX();
            odometry.b = orientation.GetY();
            odometry.c = orientation.GetZ();
            odometry.d = orientation.GetW();

            statistics.odometry_data.push_back(odometry);
        }

        m_positionPublisher.publish(statistics);

        //update ROS status
        ros::spinOnce();
    }
}

/****************************************/
/****************************************/

void CStatisticsLoopFunctions::Init(TConfigurationNode &t_tree) {
    GetNodeAttributeOrDefault(t_tree, "num_robots", m_nRobots, m_nRobots);

    // Set the number of rows and columns in the matrix
    int numRows = m_nRobots; // number of rows
    int numCols = m_nRobots; // number of columns

    count = 0;

    // Resize the matrix to the specified number of rows and columns
//    m_distanceMatrix.resize(numRows, std::vector<DistanceFactorPair>(numCols));

    InitROS();
}

/****************************************/
/****************************************/

void CStatisticsLoopFunctions::Reset() {
    /* Clear distance matrix */
//    for (int i = 0; i < m_nRobots; ++i) {
//        m_distanceMatrix[i].clear();
//    }

    count = 0;
}

/****************************************/
/****************************************/

void CStatisticsLoopFunctions::PostStep() {
//    /* Get the map of all foot-bots from the space */
//    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
//    /* Go through them */
//    for (CSpace::TMapPerType::iterator it_1 = tFBMap.begin(); it_1 != tFBMap.end(); ++it_1) {
//        /* Create a pointer to the current foot-bot */
//        CFootBotEntity *pcFB_1 = any_cast<CFootBotEntity *>(it_1->second);
//
//        for (CSpace::TMapPerType::iterator it_2 = tFBMap.begin(); it_2 != tFBMap.end(); ++it_2) {
//            /* Create a pointer to the current foot-bot */
//            CFootBotEntity *pcFB_2 = any_cast<CFootBotEntity *>(it_2->second);
//
//            if (((it_1->first)[2] - 'A') > ((it_2->first)[2] - 'A')) {
//                continue;
//            }
//
//            /* Add the current position of the foot-bot if it's sufficiently far from the last */
//            Real distance = std::sqrt(SquareDistance(
//                    pcFB_1->GetEmbodiedEntity().GetOriginAnchor().Position,
//                    pcFB_2->GetEmbodiedEntity().GetOriginAnchor().Position
//            )) * 100;
//
//            m_distanceMatrix[(int) ((it_1->first)[2] - 'A')][(int) ((it_2->first)[2] - 'A')] = std::make_pair(distance, 1);
//        }
//    }
    ControlStepROS();
    count++;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CStatisticsLoopFunctions, "statistics_loop_functions")
