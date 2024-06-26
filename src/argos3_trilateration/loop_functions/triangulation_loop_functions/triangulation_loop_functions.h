#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <iostream>
#include <vector>
#include <utility>
#include <cmath>

/* ROS dependencies */
#include "ros/ros.h"
#include <tri_msgs/Item.h>
#include <tri_msgs/Matrix.h>
#include <tri_msgs/Agent.h>

using namespace argos;

// Define a type for the pair of floats
typedef std::pair<float, float> DistanceFactorPair;

// Define a type for the distance matrix
typedef std::vector<std::vector<DistanceFactorPair>> DistanceMatrix;

class CTriangulationLoopFunctions : public CLoopFunctions {

public:

    typedef std::map<CFootBotEntity*, std::vector<CVector3> > TWaypointMap;
    TWaypointMap m_tWaypoints;

    // Map of foot-bots and their distance matrices
    typedef std::map<CFootBotEntity*, DistanceMatrix*> TDistanceMatrixMap;
    TDistanceMatrixMap m_tDistanceMatrices;

public:

    CTriangulationLoopFunctions();

    ~CTriangulationLoopFunctions() override {}

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void PostStep();

    /* ROS related methods */
    virtual void InitROS();
    virtual void ControlStepROS();

private:

    /* Distance Matrix */
    DistanceMatrix m_distanceMatrix; // distance matrix for the range and bearing communication
    int m_nRobots;

    /* ROS Publisher */
    ros::Publisher m_matrixPublisher;
    tri_msgs::Agent m_matrixMessage;

};

#endif
