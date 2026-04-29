cmake_minimum_required(VERSION 3.8)
project(wheeled_dog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED) # 这个不用添加xml，因为rclcpp已经链接了pthread库


add_executable(motor_controller_node 
src/driver/drv_unittree_motor.cpp
src/algorithm/alg_slope.cpp
)
add_executable(controller_bridge_node src/interaction/ita_controller_bridge.cpp)
add_executable(keyboard_controller_node src/device/dev_keyboard.cpp)


target_include_directories(motor_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(controller_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 
target_include_directories(keyboard_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 

target_compile_features(motor_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(controller_bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(keyboard_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  motor_controller_node
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(motor_controller_node Threads::Threads)
# 专门给电机节点分配一个线程，且不放在依赖库中，因为它是通过pthread库实现的，而不是通过rclcpp实现的



ament_target_dependencies(
  controller_bridge_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)
ament_target_dependencies(
  keyboard_controller_node
  "rclcpp"
  "geometry_msgs"
)


target_include_directories(motor_controller_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/third_party/unittree/include)
target_link_libraries(motor_controller_node ${CMAKE_CURRENT_SOURCE_DIR}/third_party/unittree/lib/libUnitreeMotorSDK_M80106_Linux64.so)
set_target_properties(motor_controller_node PROPERTIES 
  INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unittree/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)


install(TARGETS motor_controller_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS controller_bridge_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS keyboard_controller_node
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
