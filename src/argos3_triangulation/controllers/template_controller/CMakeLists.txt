add_library(templateController MODULE templateController.h templateController.cpp)
target_link_libraries(templateController
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)

