#ifndef TRAJECTORY_QTUSER_FUNCTIONS_H
#define TRAJECTORY_QTUSER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CTriangulationLoopFunctions;

class CTriangulationQTUserFunctions : public CQTOpenGLUserFunctions {

public:

    CTriangulationQTUserFunctions();

    virtual ~CTriangulationQTUserFunctions() {}

    virtual void DrawTriangulation();

private:



private:

    CTriangulationLoopFunctions& m_cTriangLF;

};

#endif
