add_library(random_walk_controller MODULE random_walk_controller.h random_walk_controller.cpp)

target_link_libraries(random_walk_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
