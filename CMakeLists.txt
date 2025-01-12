cmake_minimum_required(VERSION 3.8)
project(dbscan_segmentation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


ament_auto_find_build_dependencies()

ament_auto_add_library(dbscan_seg_component SHARED
 src/dbscan_seg_component.cpp)

rclcpp_components_register_nodes(dbscan_seg_component
  "dbscan_segmentation::DBScanNode"
)

ament_auto_add_executable(dbscan_seg_node
    NO_TARGET_LINK_LIBRARIES
    src/dbscan_seg_node.cpp
)

target_link_libraries(dbscan_seg_node
  dbscan_seg_component
)

#
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  dbscan_seg_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()











