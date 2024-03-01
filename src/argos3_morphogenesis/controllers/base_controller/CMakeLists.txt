add_library(base_controller MODULE base_controller.h base_controller.cpp)

target_link_libraries(base_controller
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
