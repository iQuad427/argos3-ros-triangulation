/* Include the controller definition */
#include "base_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

float CFootBotBase::m_distance;
float CFootBotBase::m_gradient;
float CFootBotBase::m_activation;

CFootBotBase::CFootBotBase() :
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

void CFootBotBase::InitROS() {
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
    ros::NodeHandle self_pub_node;
    ros::NodeHandle sub_node;

    std::stringstream publisherName;
    std::stringstream selfPublisherName;
    std::stringstream subscriberName;

    publisherName << name.str() << "/distances";
    selfPublisherName << name.str() << "/distance";
    subscriberName << name.str() << "/direction";

    // Register the publisher to the ROS master
    m_distancePublisher = self_pub_node.advertise<tri_msgs::Distance>(selfPublisherName.str(), 10);
    m_distancesPublisher = pub_node.advertise<tri_msgs::Distances>(publisherName.str(), 10);
    m_directionSubscriber = sub_node.subscribe(subscriberName.str(), 10, CallbackROS);
}

void CFootBotBase::CallbackROS(const morpho_msgs::Direction::ConstPtr& msg) {
    m_gradient = msg->gradient;
    m_distance = msg->distance;
    m_activation = msg->activation;
}

void CFootBotBase::ControlStepROS() {
    if (ros::ok()) {
        // Publish the message
        if (m_distancesMessage.robot_id != 0) {
            m_distancesPublisher.publish(m_distancesMessage);

            // Clean message for next iteration
            m_distancesMessage.robot_id = 0;
            m_distancesMessage.ranges.clear();
        }
        if (m_distanceMessage.other_robot_id != 0) {
            m_distancePublisher.publish(m_distanceMessage);

            // Clean the message for next iteration
            m_distanceMessage.other_robot_id = 0;
        }

        //update ROS status
        ros::spinOnce();
    }
}

/****************************************/
/****************************************/

void CFootBotBase::Init(TConfigurationNode &t_node) {
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

    // State machine
    m_state = MOVE;

    // Fill the distance table with ones
    m_distanceTable.resize(m_nRobots, DistanceFactorPair(0, 1));

    InitROS();
}

/****************************************/
/****************************************/

void CFootBotBase::Reset() {
    // Fill the table with ones
    m_distanceTable.resize(m_nRobots, DistanceFactorPair(0, 1));
}

/****************************************/
/****************************************/

void CFootBotBase::ControlStep() {
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

        // Save the estimated range (just measured)
        if (initiator_id != '\0' && responder_id != '\0') {
            // TODO: could also want to replace by average of previous and current to even out the result
            m_distanceTable[(int) responder_id - 'A'] = std::make_pair(tPackets[un_SelectedPacket].Range, 1);

            m_distanceMessage.other_robot_id = (int) responder_id;
            m_distanceMessage.distance = tPackets[un_SelectedPacket].Range;
            m_distanceMessage.certainty = 1.0f;
        }

        float range;

        tri_msgs::Distance item;

        m_distancesMessage.robot_id = (int) responder_id;

        for (int k = 0; k < m_nRobots; ++k) {
            data >> range;

            item.other_robot_id = 'A' + k;
            item.distance = (Real) range;
            item.certainty = (float) 1.0f; // TODO add certainty factor (should come from other robot) => data >> certainty

            m_distancesMessage.ranges.push_back(item);
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
     * 3. Information (distance update)
     */
    CByteArray cMessage;
    cMessage << (UInt8) 'A';          // ID of sender ('A' is the broadcast ID)
    cMessage << (UInt8) GetId()[2];   // ID of receiver

    for (uint16_t j = 0; j < m_nRobots; ++j) { // 32 bit = 4 bytes => requires 10 * 4 = 40 bytes of data for 10 robots
        cMessage << (float) m_distanceTable[j].first;
    }

    // Fill to the size of communication allowed (bytes)
    cMessage.Resize(m_unBandWidth, '\0');

    // Send the message
    m_pcRangeAndBearingActuator->SetData(cMessage);

    /** State Machine based Movement */

//    std::cout << m_state << std::endl;
//    std::cout << m_gradient << std::endl;

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
REGISTER_CONTROLLER(CFootBotBase, "base_controller")
