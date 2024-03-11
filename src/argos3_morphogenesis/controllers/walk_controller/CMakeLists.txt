add_library(walk_controller MODULE walk_controller.h walk_controller.cpp)

target_link_libraries(walk_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
