add_library(test MODULE triangulation_controller.h triangulation_controller.cpp)

target_link_libraries(test
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
