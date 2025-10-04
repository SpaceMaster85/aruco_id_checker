cmake_minimum_required(VERSION 3.5)
project(aruco_id_checker)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(aruco_markers_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource index
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

# Install executables
install(PROGRAMS
  ${PROJECT_NAME}/pose_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
