#ifndef TRAJECTORY_LOOP_FUNCTIONS_H
#define TRAJECTORY_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

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

    virtual ~CTriangulationLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);

    virtual void Reset();

    virtual void PostStep();

private:

};

#endif
