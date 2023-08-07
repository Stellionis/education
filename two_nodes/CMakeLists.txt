cmake_minimum_required(VERSION 3.8)
project(two_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub src/pub.cpp)
add_executable(sub src/sub.cpp)

ament_target_dependencies(sub rclcpp std_msgs)
ament_target_dependencies(pub rclcpp std_msgs)

target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pub PUBLIC c_std_99 cxx_std_17) 
target_compile_features(sub PUBLIC c_std_99 cxx_std_17) 


install(TARGETS 
  pub
  sub
  DESTINATION lib/${PROJECT_NAME}
  )
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
