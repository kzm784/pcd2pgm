cmake_minimum_required(VERSION 3.5)
project(pcd2pgm)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(pcd2pgm src/pcd2pgm.cpp)

# Specify dependencies
ament_target_dependencies(pcd2pgm
  rclcpp 
  PCL
  OpenCV
)

# Link libraries (not necessary if ament_target_dependencies is used correctly)
target_link_libraries(pcd2pgm
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS pcd2pgm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
