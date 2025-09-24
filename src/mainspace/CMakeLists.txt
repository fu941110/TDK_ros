cmake_minimum_required(VERSION 3.8)
project(mainspace)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 若未來使用 geometry 轉換
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToStmSpeed.msg"
  "msg/Position.msg"
  "msg/Desk.msg"
  "msg/Coffee.msg"
  "msg/CsvFile.msg"
  "msg/Command.msg"
  "msg/Pause.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(navigator_lib
  src/navigator.cpp
)

# add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(navigator_lib
  ${cpp_typesupport_target}
)
# install(TARGETS navigator
#   DESTINATION lib/${PROJECT_NAME}
# )

# === 共用的依賴設定 ===
set(NODE_DEPENDENCIES
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# === 要建立的 node 清單 ===
set(NODE_SOURCES
  navigator_main
  Stage2
  Stage3
  Stage4
  StageManager
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS navigator_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(node ${NODE_SOURCES})
  add_executable(${node} src/${node}.cpp)
  ament_target_dependencies(${node} ${NODE_DEPENDENCIES})
  target_link_libraries(${node} navigator_lib ${cpp_typesupport_target})
  install(TARGETS ${node}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# === 安裝資料夾 ===
install(DIRECTORY MainPath_csv/
  DESTINATION share/${PROJECT_NAME}/MainPath_csv
)

install(DIRECTORY Stage2Path_csv/
  DESTINATION share/${PROJECT_NAME}/Stage2Path_csv
)

install(DIRECTORY Stage2Path_easy_csv/
  DESTINATION share/${PROJECT_NAME}/Stage2Path_easy_csv
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
