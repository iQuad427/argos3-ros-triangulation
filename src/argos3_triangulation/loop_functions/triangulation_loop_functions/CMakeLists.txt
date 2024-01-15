include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
add_library(triangulation_loop_function MODULE
        triangulation_loop_functions.h
        triangulation_loop_functions.cpp
        triangulation_qtuser_functions.h
        triangulation_qtuser_functions.cpp)

target_link_libraries(triangulation_loop_function
        argos3core_simulator
        argos3plugin_simulator_entities
        argos3plugin_simulator_footbot
        argos3plugin_simulator_qtopengl
        ${ARGOS_QTOPENGL_LIBRARIES})
