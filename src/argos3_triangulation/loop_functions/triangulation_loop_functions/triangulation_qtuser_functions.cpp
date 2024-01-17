#include "triangulation_qtuser_functions.h"
#include "triangulation_loop_functions.h"

/****************************************/
/****************************************/

CTriangulationQTUserFunctions::CTriangulationQTUserFunctions() :
        m_cTriangLF(dynamic_cast<CTriangulationLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
}


/****************************************/
/****************************************/

void CTriangulationQTUserFunctions::DrawTriangulation() {}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTriangulationQTUserFunctions, "triangulation_qtuser_functions")
