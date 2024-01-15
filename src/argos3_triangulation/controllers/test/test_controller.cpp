/* Include the controller definition */
#include "test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
        m_pcWheels(nullptr),
        m_pcProximity(nullptr),
        m_pcRangeAndBearingSensor(nullptr),
        m_pcRangeAndBearingActuator(nullptr),
        m_unBandWidth(10),
        m_cAlpha(10.0f),
        m_fDelta(0.5f),
        m_fWheelVelocity(2.5f),
        m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                                ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode &t_node) {
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

    // Set the number of rows and columns in the matrix
    int numRows = 10; // Set your desired number of rows
    int numCols = 10; // Set your desired number of columns

    // Resize the matrix to the specified number of rows and columns
    m_distanceMatrix.resize(numRows, std::vector<DistanceFactorPair>(numCols));

//    for (int i = 0; i < numRows; ++i) {
//        for (int j = 0; j < numCols; ++j) {
//            float distance = 0.0f; // Replace with your actual distance value
//            float factor = 0.0f;   // Replace with your actual factor value
//            m_distanceMatrix[i][j] = std::make_pair(distance, factor);
//        }
//    }
//    // Access and print the elements of the matrix
//    for (int i = 0; i < numRows; ++i) {
//        for (int j = 0; j < numCols; ++j) {
//            std::cout << "(" << m_distanceMatrix[i][j].first << ", " << m_distanceMatrix[i][j].second << ") ";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << "------------------------------------" << std::endl;
}

/****************************************/
/****************************************/

void CFootBotDiffusion::Reset() {
    // Reset the matrix to the specified number of rows and columns
    int numRows = 10; // Set your desired number of rows
    int numCols = 10; // Set your desired number of columns

    // Resize the matrix to the specified number of rows and columns
    m_distanceMatrix.resize(numRows, std::vector<DistanceFactorPair>(numCols));
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
    /* Simulate Communication */

    /** Initiator */
    /* Note: only simulating the reception of the acknowledgment message, which is sent by the responder
     *       after receiving the message from the initiator.
     *       => receives the message that allows for distance estimation
     */

    // Get readings from range and bearing sensor
    const CCI_RangeAndBearingSensor::TReadings &tPackets = m_pcRangeAndBearingSensor->GetReadings();

    uint8_t buffer[1] = {'\0'};

    uint8_t initiator_id[1] = {'\0'};
    uint8_t responder_id[1] = {'\0'};

    if (!tPackets.empty()) {
        size_t un_SelectedPacket = CRandom::CreateRNG("argos")->Uniform(CRange<UInt32>(0, tPackets.size()));

        for (size_t j = 0; j < tPackets[un_SelectedPacket].Data.Size(); ++j) {
            buffer[0] = (uint8_t) tPackets[un_SelectedPacket].Data[j];
            if (buffer[0] == '\0') {
                continue; // Skip null characters, avoid losing time
            }

            if (j == 0 && buffer[0] == 'A') {
                // Consider all messages received as requested messages (since don't simulate the polling message)
                initiator_id[0] = (uint8_t) GetId()[2];
                buffer[0] = '\0';
            } else if (j == 1 && buffer[0] != (uint8_t) GetId()[2]) {
                // Avoid receiving messages from itself
                responder_id[0] = buffer[0];
                buffer[0] = '\0';
            }
        }

        // Update the distance matrix
        int x = 0;
        int y = 0;
        int i = 0;
        int j = 0;
        if (initiator_id[0] != '\0' && responder_id[0] != '\0') {
            // Find indexes of initiator and responder using - '0'
            i = (int) initiator_id[0] - '0';
            j = (int) responder_id[0] - '0';

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
    }

    // Print the distance matrix if id = 0
//    if (GetId()[2] == '5') {
//        for (int i = 0; i < m_distanceMatrix.size(); ++i) {
//            for (int j = 0; j < m_distanceMatrix[i].size(); ++j) {
//                std::cout << "(" << m_distanceMatrix[i][j].first << ", " << m_distanceMatrix[i][j].second << ") ";
//            }
//            std::cout << std::endl;
//        }
//        std::cout << "------------------------------------" << std::endl;
//    }

    // Reset buffers
    buffer[0] = '\0';
    initiator_id[0] = '\0';
    responder_id[0] = '\0';

    /** Responder */
    /*  Note: only simulating the sending of the acknowledgment message, which is sent by the responder
     *        after receiving the message from the initiator.
     *        => sends the message that allows for distance estimation
     */

    /* Send a message
     * 1. ID of sender (robot that is supposed to receive the message)
     * 2. ID of receiver (itself)
     * 3. TODO: Information (distance matrix update)
     */
    CByteArray cMessage;
    cMessage << (uint8_t) 'A';          // ID of sender ('A' is the broadcast ID)
    cMessage << (uint8_t) GetId()[2];   // ID of receiver
    // TODO: Information (distance matrix update)

    // Fill to the size of communication allowed (bytes)
    cMessage.Resize(m_unBandWidth, '\0');

    // Print message
    for (size_t i = 0; i < cMessage.Size(); ++i) {
        buffer[0] = (char) cMessage[i];
        if (buffer[0] == '\0') {
            continue; // Skip null characters, avoid losing time
        }

        printf("Sending %c, at index %zu\n", buffer[0], i);
    }

    // Send the message
    m_pcRangeAndBearingActuator->SetData(cMessage);

    /** Log Information */

//    if (GetId()[2] == '0') {
//        std::cout << tPackets.size() << std::endl;
//    }

    /** Avoid Obstacles */

    /* Get readings from proximity sensor */
    const CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    /* Sum them together */
    CVector2 cAccumulator;
    for (auto tProxRead: tProxReads) {
        cAccumulator += CVector2(tProxRead.Value, tProxRead.Angle);
    }
    cAccumulator /= tProxReads.size();
    /* If the angle of the vector is small enough and the closest obstacle
     * is far enough, continue going straight, otherwise curve a little
     */
    CRadians cAngle = cAccumulator.Angle();
    if (m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
        cAccumulator.Length() < m_fDelta) {
        /* Go straight */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    } else {
        /* Turn, depending on the sign of the angle */
        if (cAngle.GetValue() > 0.0f) {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
        } else {
            m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
        }
    }
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
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
