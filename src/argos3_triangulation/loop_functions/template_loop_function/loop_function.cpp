
#include "loop_function.h"

// Best source: Red walls --> DARK is best
//              Blue walls --> LIGHT is best

/****************************************/
/****************************************/
/*static const Real BORDER_W_MINX            = -2.5f;
static const Real BORDER_W_MAXX            = -2.0f;
static const Real BORDER_W_MINY            = -2.5f;
static const Real BORDER_W_MAXY            = 2.5f;

static const Real BORDER_E_MINX            = 2.0f;
static const Real BORDER_E_MAXX            = 2.5f;
static const Real BORDER_E_MINY            = -2.5f;
static const Real BORDER_E_MAXY            = 2.5f;

static const Real BORDER_S_MINX            = -2.5f;
static const Real BORDER_S_MAXX            = 2.5f;
static const Real BORDER_S_MINY            = -2.5f;
static const Real BORDER_S_MAXY            = -2.0f;

static const Real BORDER_N_MINX            = -2.5f;
static const Real BORDER_N_MAXX            = 2.5f;
static const Real BORDER_N_MINY            = 2.0f;
static const Real BORDER_N_MAXY            = 2.5f;*/

static const Real SOURCE_RADIUS       = 0.25f;
static const Real SOURCE_X          = 0.0f;
static const Real SOURCE_Y          = 0.0f;

static const Real OBJECT_RADIUS          = 0.03f;
static const Real OBJECT_DIAMETER        = OBJECT_RADIUS * 2.0f;
static const Real OBJECT_HEIGHT        = 0.15f;

#define PI 3.14159265


/****************************************/
/****************************************/

CLoopTemplateController::CLoopTemplateController() {
  
}

/****************************************/
/****************************************/

CLoopTemplateController::~CLoopTemplateController() {
    RemoveArena();
}

/****************************************/
/****************************************/

void CLoopTemplateController::Init(TConfigurationNode& t_tree) {
    /* Get output file name from XML tree */
    GetNodeAttribute(t_tree, "output", m_strOutFile);
    /* Open the file for text writing */
    m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
    if(m_cOutFile.fail()) {
       THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
    }

    m_pcRNG = CRandom::CreateRNG("argos");

    PositionArena();
    InitializeArena();

}

/****************************************/
/****************************************/

void CLoopTemplateController::InitializeArena() {
    m_cCoordSource = CVector2(SOURCE_X, SOURCE_Y);
}

/****************************************/
/****************************************/

void CLoopTemplateController::Reset() {
    /* Close the output file */
    m_cOutFile.close();
    if(m_cOutFile.fail()) {
        THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
    }
    /* Open the file for text writing */
    m_cOutFile.open(m_strOutFile.c_str(), std::ofstream::out | std::ofstream::trunc);
    if(m_cOutFile.fail()) {
        THROW_ARGOSEXCEPTION("Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
    }

    InitializeArena();

}

/****************************************/
/****************************************/

void CLoopTemplateController::Destroy() {
    /* Close the output file */
    m_cOutFile.close();
    if(m_cOutFile.fail()) {
        THROW_ARGOSEXCEPTION("Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
    }
}

/****************************************/
/****************************************/

void CLoopTemplateController::PreStep() {

  CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");
  UInt8 unRobotId;
  for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it) {
     /* Get handle to e-puck entity and controller */
     CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
     /* Get the position of the e-puck on the ground as a CVector2 */
     CVector2 cPos;
     cPos.Set(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
     unRobotId = atoi(cEPuck.GetId().substr(2, 3).c_str());
  }

  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    /*CVector3 cEpuckPosition(0,0,0);
   
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {

        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);

        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY(),
                           2*acos(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW()));
                           }
                          
         LOG << "--------------" << cEpuckPosition.GetX() << std::endl;
                                                        
         LOG << "X: " << cEpuckPosition.GetX() << std::endl;
         LOG << "Y:" << cEpuckPosition.GetY() << std::endl;
         LOG << "Theta:" << cEpuckPosition.GetZ() << std::endl;*/
}

/****************************************/
/****************************************/

void CLoopTemplateController::PostStep() {}

/****************************************/
/****************************************/

void CLoopTemplateController::PostExperiment() {}

/****************************************/
/****************************************/

CColor CLoopTemplateController::GetFloorColor(const CVector2& c_position_on_plane) 
{
    /* Border area is black.*/ 
    /*if(c_position_on_plane.GetX() >= BORDER_W_MINX && c_position_on_plane.GetX() <= BORDER_W_MAXX &&
            c_position_on_plane.GetY() >= BORDER_W_MINY && c_position_on_plane.GetY() <= BORDER_W_MAXY) {
        return CColor::BLACK;
    }
    if(c_position_on_plane.GetX() >= BORDER_E_MINX && c_position_on_plane.GetX() <= BORDER_E_MAXX &&
            c_position_on_plane.GetY() >= BORDER_E_MINY && c_position_on_plane.GetY() <= BORDER_E_MAXY) {
        return CColor::BLACK;
    }
    if(c_position_on_plane.GetX() >= BORDER_N_MINX && c_position_on_plane.GetX() <= BORDER_N_MAXX &&
            c_position_on_plane.GetY() >= BORDER_N_MINY && c_position_on_plane.GetY() <= BORDER_N_MAXY) {
        return CColor::BLACK;
    }
    if(c_position_on_plane.GetX() >= BORDER_S_MINX && c_position_on_plane.GetX() <= BORDER_S_MAXX &&
            c_position_on_plane.GetY() >= BORDER_S_MINY && c_position_on_plane.GetY() <= BORDER_S_MAXY) {
        return CColor::BLACK;
    }*/
    /* Source */
    CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
    Real d = (m_cCoordSource - vCurrentPoint).Length();
    /*if (d <= SOURCE_RADIUS) 
    {
    	return CColor::BLACK;
    }*/
    /* Rest of the arena is white. */
    return CColor::WHITE;
}

/****************************************/
/****************************************/

bool CLoopTemplateController::IsOnSource(CVector2& c_position_robot) {
  if ((m_cCoordSource - c_position_robot).Length() <= SOURCE_RADIUS) {
    return true;
  }
  return false;
}

/****************************************/
/****************************************/

void CLoopTemplateController::PositionArena() {
  CArenaEntity* pcArena;
    pcArena = new CArenaEntity("arena",
                               CVector3(0,0,0),
                               CQuaternion().FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO),
                               CVector3(0.001,1,0.06),
                               "leds",
                               1,
                               6,
                               0.017f,
                               1.0f);
    AddEntity(*pcArena);
    m_pcArena = pcArena;
}

/****************************************/
/****************************************/

void CLoopTemplateController::RemoveArena() {
    std::ostringstream id;
    id << "arena";
    RemoveEntity(id.str().c_str());
}

/****************************************/
/****************************************/

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CLoopTemplateController, "template_controller_loop_function");
