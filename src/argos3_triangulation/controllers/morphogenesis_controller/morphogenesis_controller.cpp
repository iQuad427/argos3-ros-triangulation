/* Include the controller definition */
#include "morphogenesis_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

float CFootBotMorphogenesis::m_distance;
float CFootBotMorphogenesis::m_gradient;
float CFootBotMorphogenesis::m_activation;

CFootBotMorphogenesis::CFootBotMorphogenesis() :
        m_pcWheels(nullptr),
        m_pcProximity(nullptr),
        m_pcRangeAndBearingSensor(nullptr),
        m_pcRangeAndBearingActuator(nullptr),
        m_unBandWidth(10),
        m_cAlpha(10.0f),
        m_fDelta(0.5f),
        m_fWheelVelocity(2.5f),
        m_nRobots(10),
        m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                                ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotMorphogenesis::InitROS() {
    //get e-puck ID
    std::stringstream name;
    name.str("");
    name << GetId(); // fbX

    //init ROS
    if (!ros::isInitialized()) {
        char **argv = NULL;
        int argc = 0;
        ros::init(argc, argv, name.str());
    }

    // ROS access node
    ros::NodeHandle pub_node;
    ros::NodeHandle sub_node;

    std::stringstream publisherName;
    std::stringstream subscriberName;

    publisherName << name.str() << "/distance_matrix";
    subscriberName << name.str() << "/direction";

    // Register the publisher to the ROS master
    m_matrixPublisher = pub_node.advertise<tri_msgs::Agent>(publisherName.str(), 10);
    m_directionSubscriber = sub_node.subscribe(subscriberName.str(), 10, CallbackROS);

    // Prefill Messages
    m_matrixMessage.header.frame_id = publisherName.str();
    m_matrixMessage.agent_id = (uint8_t) GetId()[2];

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

void CFootBotMorphogenesis::CallbackROS(const morpho_msgs::Direction::ConstPtr& msg) {
    m_gradient = msg->gradient;
    m_distance = msg->distance;
    m_activation = msg->activation;
}

void CFootBotMorphogenesis::ControlStepROS() {
    if (ros::ok()) {
        std::stringstream name;
        name.str("");
        name << GetId()[2];

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

void CFootBotMorphogenesis::Init(TConfigurationNode &t_node) {
    // Get sensor/actuator handles
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

    // Add the range and bearing sensor and actuator
    m_pcRangeAndBearingSensor = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcRangeAndBearingActuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");

    // Parse the configuration file
    GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
    m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
    GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "size", m_unBandWidth, m_unBandWidth);

    GetNodeAttributeOrDefault(t_node, "num_robots", m_nRobots, m_nRobots);

    // Set the number of rows and columns in the matrix
    int numRows = m_nRobots; // number of rows
    int numCols = m_nRobots; // number of columns

    // State machine
    m_state = MOVE;

    // Resize the matrix to the specified number of rows and columns
    m_distanceMatrix.resize(numRows, std::vector<DistanceFactorPair>(numCols));

    InitROS();
}

/****************************************/
/****************************************/

DistanceMatrix *CFootBotMorphogenesis::GetDistanceMatrix() {
    return &m_distanceMatrix;
}

/****************************************/
/****************************************/

void CFootBotMorphogenesis::Reset() {
    // Reset the matrix to the specified number of rows and columns
    int numRows = m_nRobots; // Set your desired number of rows
    int numCols = m_nRobots; // Set your desired number of columns

    // Resize the matrix to the specified number of rows and columns
    m_distanceMatrix.resize(numRows, std::vector<DistanceFactorPair>(numCols));
}

/****************************************/
/****************************************/

void CFootBotMorphogenesis::ControlStep() {
    /* Simulate Communication */

    /** Initiator */
    /* Note: only simulating the reception of the acknowledgment message, which is sent by the responder
     *       after receiving the message from the initiator.
     *       => receives the message that allows for distance estimation
     */

    // Get readings from range and bearing sensor
    const CCI_RangeAndBearingSensor::TReadings &tPackets = m_pcRangeAndBearingSensor->GetReadings();

    UInt8 initiator_id;
    UInt8 responder_id;

    if (!tPackets.empty()) {
        size_t un_SelectedPacket = CRandom::CreateRNG("argos")->Uniform(CRange<UInt32>(0, tPackets.size()));

        CByteArray data = tPackets[un_SelectedPacket].Data;

        data >> initiator_id;
        data >> responder_id;

        // Update the distance matrix
        int x;
        int y;
        int i;
        int j;
        if (initiator_id != '\0' && responder_id != '\0') {
            // Find indexes of initiator and responder using - 'A'
            i = (int) GetId()[2] - 'A';
            j = (int) responder_id - 'A';

            if (i < j) {
                x = i;
                y = j;
            } else {
                x = j;
                y = i;
            }

            // Update the distance matrix
            m_distanceMatrix[x][y] = std::make_pair(tPackets[un_SelectedPacket].Range, 1);
        }

        std::cout << GetId()[2] << std::endl;

        if (GetId()[2] == 'A') {
            float range;
            int id = (int) responder_id - 'A';

            for (int k = 0; k < m_nRobots; ++k) {
                data >> range;

                if (id < k) {
                    x = id;
                    y = k;
                } else {
                    x = k;
                    y = id;
                }

                m_distanceMatrix[x][y] = std::make_pair<Real, float>((Real) range, (float) 1);
            }
        }
    }

    /** Responder */
    /*  Note: only simulating the sending of the acknowledgment message, which is sent by the responder
     *        after receiving the message from the initiator.
     *        => sends the message that allows for distance estimation
     */

    /* Send a message
     * 1. ID of sender (robot that is supposed to receive the message)
     * 2. ID of receiver (itself)
     * 3. Information (distance matrix update)
     */
    CByteArray cMessage;
    cMessage << (UInt8) 'A';          // ID of sender ('A' is the broadcast ID)
    cMessage << (UInt8) GetId()[2];   // ID of receiver

    int agent_id = (int) GetId()[2] - 'A';

    int x;
    int y;
    for (uint16_t j = 0; j < m_nRobots; ++j) { // 32 bit = 4 bytes => requires 10 * 4 = 40 bytes of data
        if (j < agent_id) {
            x = j;
            y = agent_id;
        } else {
            x = agent_id;
            y = j;
        }

        cMessage << (float) m_distanceMatrix[x][y].first;
    }

    // Fill to the size of communication allowed (bytes)
    cMessage.Resize(m_unBandWidth, '\0');

    // Send the message
    m_pcRangeAndBearingActuator->SetData(cMessage);

    /** State Machine based Movement */

//    std::cout << m_counter << std::endl;
    std::cout << m_gradient << std::endl;
    std::cout << m_activation << std::endl;
//    std::cout << m_distance << std::endl;

    /* State Transitions */
    if (m_state == MOVE) {
        if (m_distance > 5) {
            if (m_gradient < 0) {
                m_state = TURN; // Should try another direction
                m_counter = m_activation * 100;
            }
        } else {
            m_state = STOP;
        }
    } else if (m_state == TURN) {
        if (m_counter <= 0) {
            m_state = GO;
            m_counter = 20;
        } else {
            m_counter--;
        }
    } else if (m_state == GO) {
        if (m_counter <= 0) {
            m_state = MOVE;
        } else {
            m_counter--;
        }
    } else if (m_state == STOP) {
        if (m_distance > 5) {
            m_state = MOVE;
        }
    }

    /* State Action */
    switch (m_state) {
        case MOVE :
//            std::cout << m_state << ": moving forward" << std::endl;
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            break;
        case GO:
//            std::cout << m_state << ": going forward" << std::endl;
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            break;
        case TURN:
//            std::cout << m_state << ": turning" << std::endl;
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
            break;
        case STOP:
//            std::cout << m_state << ": stopped" << std::endl;
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            break;
    }

    ControlStepROS();
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotMorphogenesis, "footbot_morphogenesis_controller")
