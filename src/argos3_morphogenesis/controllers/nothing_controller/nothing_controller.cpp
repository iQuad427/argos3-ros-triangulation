/* Include the controller definition */
#include "nothing_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotBase::CFootBotBase() {}

/****************************************/
/****************************************/

/****************************************/
/****************************************/

void CFootBotBase::Init(TConfigurationNode &t_node) {}

/****************************************/
/****************************************/

void CFootBotBase::Reset() {}

/****************************************/
/****************************************/

void CFootBotBase::ControlStep() {}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotBase, "nothing_controller")
