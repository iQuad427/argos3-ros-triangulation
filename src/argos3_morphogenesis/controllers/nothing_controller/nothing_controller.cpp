/* Include the controller definition */
#include "nothing_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/****************************************/
/****************************************/

CFootBotBase::CFootBotBase() {
    std::cout << "Creation" << std::endl;
}

/****************************************/
/****************************************/

/****************************************/
/****************************************/

void CFootBotBase::Init(TConfigurationNode &t_node) {
    std::cout << "Initialize" << std::endl;
}

/****************************************/
/****************************************/

void CFootBotBase::Reset() {
    std::cout << "Reset" << std::endl;
}

/****************************************/
/****************************************/

void CFootBotBase::ControlStep() {
    std::cout << "Control Step" << std::endl;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CFootBotBase, "nothing_controller")
