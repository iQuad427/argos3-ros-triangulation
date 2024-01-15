add_library(loop_func SHARED loop_function.h loop_function.cpp)
target_link_libraries(loop_func
  ${ARGOS_LIBRARIES}
  argos3plugin_simulator_entities
  argos3plugin_simulator_epuck
  argos3plugin_simulator_media
  argos3plugin_simulator_arena)

