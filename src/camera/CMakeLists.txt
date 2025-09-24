cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mainspace REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# add_executable(cameraBackground src/cameraBackground.cpp)
# ament_target_dependencies(cameraBackground OpenCV)

# add_executable(cameraTest src/cameraTest.cpp)
# ament_target_dependencies(cameraTest OpenCV)

add_executable(cameraCoffee2 src/cameraCoffee2.cpp src/cameraBackground.cpp)
ament_target_dependencies(cameraCoffee2 OpenCV rclcpp mainspace) 

add_executable(cameraStage3 src/cameraStage3.cpp src/cameraBackground.cpp)
ament_target_dependencies(cameraStage3 OpenCV rclcpp mainspace)

add_executable(cameraDesk2 src/cameraDesk2.cpp src/cameraBackground.cpp)
ament_target_dependencies(cameraDesk2 OpenCV rclcpp mainspace)

add_executable(cameraDesk2_test src/cameraDesk2_test.cpp src/cameraBackground.cpp)
ament_target_dependencies(cameraDesk2_test OpenCV rclcpp mainspace)

add_executable(cameraCoffee2_test src/cameraCoffee2_test.cpp src/cameraBackground.cpp)
ament_target_dependencies(cameraCoffee2_test OpenCV rclcpp mainspace)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS 
  # cameraBackground
  # cameraTest
  cameraCoffee2
  cameraCoffee2_test
  cameraDesk2
  cameraDesk2_test
  cameraStage3
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
