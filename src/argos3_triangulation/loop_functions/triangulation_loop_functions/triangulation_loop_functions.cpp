#include "triangulation_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3_triangulation/controllers/triangulation_controller/triangulation_controller.h>

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

CTriangulationLoopFunctions::CTriangulationLoopFunctions() :
        m_nRobots(10) {}

/****************************************/
/****************************************/

void CTriangulationLoopFunctions::InitROS() {
    //get e-puck ID
    std::stringstream name;
    name.str("");
    name << "loop_function"; // fbX

    //init ROS
    if (!ros::isInitialized()) {
        char **argv = NULL;
        int argc = 0;
        ros::init(argc, argv, name.str());
    }

    //ROS access node
    ros::NodeHandle node;

    std::stringstream publisherName;

    publisherName << name.str() << "/distance_matrix";

    // Register the publisher to the ROS master
    m_matrixPublisher = node.advertise<tri_msgs::Agent>(publisherName.str(), 10);

    // Prefill Messages
    m_matrixMessage.header.frame_id = publisherName.str();
    m_matrixMessage.agent_id = (uint8_t) 0;
    m_matrixMessage.translate = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T'};

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

void CTriangulationLoopFunctions::ControlStepROS() {
    if (ros::ok()) {
        std::stringstream name;
        name.str("");
        name << "loop_function";

        /* Fill in a message and publish using the publisher node */
        m_matrixMessage.header.stamp = ros::Time::now();

        // Add the matrix
        tri_msgs::Item item;

        // TODO: optimize data update, avoid creating everything from scratch each time
        m_matrixMessage.distance_matrix.data.clear();

        // Access and print the elements of the matrix
        for (int i = 0; i < m_nRobots; ++i) {
            for (int j = 0; j < m_nRobots; ++j) {
                item.distance = m_distanceMatrix[i][j].first;
                item.discount = m_distanceMatrix[i][j].second;
                m_matrixMessage.distance_matrix.data.push_back(item);
            }
        }

        m_matrixPublisher.publish(m_matrixMessage);

        //update ROS status
        ros::spinOnce();
    }
}

/****************************************/
/****************************************/

void CTriangulationLoopFunctions::Init(TConfigurationNode &t_tree) {
    GetNodeAttributeOrDefault(t_tree, "num_robots", m_nRobots, m_nRobots);

    std::cout << m_nRobots << std::endl;

    // Set the number of rows and columns in the matrix
    int numRows = m_nRobots; // number of rows
    int numCols = m_nRobots; // number of columns

    // Resize the matrix to the specified number of rows and columns
    m_distanceMatrix.resize(numRows, std::vector<DistanceFactorPair>(numCols));

    InitROS();
}

/****************************************/
/****************************************/

void CTriangulationLoopFunctions::Reset() {
    /* Clear distance matrix */
    for (int i = 0; i < m_nRobots; ++i) {
        m_distanceMatrix[i].clear();
    }

    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it = tFBMap.begin();
         it != tFBMap.end();
         ++it) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB = any_cast<CFootBotEntity *>(it->second);
        // Get Position : pcFB->GetEmbodiedEntity().GetOriginAnchor().Position
    }
}

/****************************************/
/****************************************/

void CTriangulationLoopFunctions::PostStep() {
    /* Get the map of all foot-bots from the space */
    CSpace::TMapPerType &tFBMap = GetSpace().GetEntitiesByType("foot-bot");
    /* Go through them */
    for (CSpace::TMapPerType::iterator it_1 = tFBMap.begin(); it_1 != tFBMap.end(); ++it_1) {
        /* Create a pointer to the current foot-bot */
        CFootBotEntity *pcFB_1 = any_cast<CFootBotEntity *>(it_1->second);

        for (CSpace::TMapPerType::iterator it_2 = tFBMap.begin(); it_2 != tFBMap.end(); ++it_2) {
            /* Create a pointer to the current foot-bot */
            CFootBotEntity *pcFB_2 = any_cast<CFootBotEntity *>(it_2->second);

            /* Add the current position of the foot-bot if it's sufficiently far from the last */
            Real distance = SquareDistance(
                    pcFB_1->GetEmbodiedEntity().GetOriginAnchor().Position,
                    pcFB_2->GetEmbodiedEntity().GetOriginAnchor().Position
            );

            m_distanceMatrix[(int) ((it_1->first)[2] - 'A')][(int) ((it_2->first)[2] - 'A')] = std::make_pair(distance, 1);

//            std::cout << it_1->first << " " << it_2->first << " " << distance << std::endl;
        }
    }
    ControlStepROS();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTriangulationLoopFunctions, "triangulation_loop_functions")
