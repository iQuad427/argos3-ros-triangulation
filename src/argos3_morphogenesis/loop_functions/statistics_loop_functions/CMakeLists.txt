include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})

add_library(statistics_loop_functions MODULE
        statistics_loop_functions.h
        statistics_loop_functions.cpp)

target_link_libraries(statistics_loop_functions
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_qtopengl
        ${ARGOS_QTOPENGL_LIBRARIES})
