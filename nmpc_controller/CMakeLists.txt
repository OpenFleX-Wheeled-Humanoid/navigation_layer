cmake_minimum_required(VERSION 3.8)
project(nmpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# acados is provided by openflex-acados_*.deb.
set(ACADOS_DIR "/opt/openflex/acados")

message(STATUS "Using acados: ${ACADOS_DIR}")

if(NOT EXISTS "${ACADOS_DIR}/include" OR NOT EXISTS "${ACADOS_DIR}/lib/libacados.so")
  message(FATAL_ERROR
    "acados install not found or incomplete: ${ACADOS_DIR}. "
    "Install openflex-acados_*.deb and run ldconfig."
  )
endif()

# Generated solver directory
set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")

# Collect generated C sources (exclude main_*.c which have their own main())
file(GLOB ACADOS_GEN_SOURCES
  "${GENERATED_DIR}/acados_solver_swerve_model.c"
  "${GENERATED_DIR}/swerve_model_model/*.c"
  "${GENERATED_DIR}/swerve_model_cost/*.c"
)

# Main library
add_library(${PROJECT_NAME} SHARED
  src/nmpc_controller.cpp
  src/acados_wrapper.cpp
  ${ACADOS_GEN_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${ACADOS_DIR}/include"
  "${ACADOS_DIR}/include/acados"
  "${ACADOS_DIR}/include/blasfeo/include"
  "${ACADOS_DIR}/include/hpipm/include"
  "${GENERATED_DIR}"
)

target_link_directories(${PROJECT_NAME} PUBLIC
  "${ACADOS_DIR}/lib"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ACADOS_DIR}/lib"
  INSTALL_RPATH "${ACADOS_DIR}/lib"
)

target_link_libraries(${PROJECT_NAME}
  acados
  hpipm
  blasfeo
  qpOASES_e
  m
)

set(DEPS
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(${PROJECT_NAME} ${DEPS})

pluginlib_export_plugin_description_file(nav2_core nmpc_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS script/generate_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})

ament_package()
