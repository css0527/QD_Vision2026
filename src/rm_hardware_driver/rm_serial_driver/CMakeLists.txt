cmake_minimum_required(VERSION 3.10)
project(rm_serial_driver)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::serial_driver::SerialDriverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN qd::serial_driver::VirtualSerialNode
  EXECUTABLE virtual_serial_node
)

target_include_directories(${PROJECT_NAME}_node PUBLIC include)

# 独立验证工具只依赖帧编解码，不启动视觉或串口 ROS 节点。
find_package(Threads REQUIRED)
add_executable(serial_probe tools/serial_probe.cpp src/float_frame.cpp)
target_include_directories(serial_probe PRIVATE include)
target_link_libraries(serial_probe Threads::Threads)
install(TARGETS serial_probe DESTINATION lib/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)

  ament_add_gtest(test_fixed_packet_tool test/test_fixed_packet_tool.cpp)
  target_link_libraries(test_fixed_packet_tool ${PROJECT_NAME})

  ament_add_gtest(test_float_frame test/test_float_frame.cpp src/float_frame.cpp)
  target_include_directories(test_float_frame PRIVATE include)

  ament_add_gtest(test_seasky_protocol test/test_seasky_protocol.cpp)
  target_link_libraries(test_seasky_protocol ${PROJECT_NAME})

  ament_add_gtest(test_serial_probe test/test_serial_probe.cpp)
  set_tests_properties(test_serial_probe PROPERTIES
    ENVIRONMENT "SERIAL_PROBE_EXE=$<TARGET_FILE:serial_probe>"
  )
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
)
