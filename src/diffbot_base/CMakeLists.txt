cmake_minimum_required(VERSION 3.8)
project(diffbot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diffbot_hardware REQUIRED)


include_directories(
  include
)

add_library(test1 
 src/diffbot_node.cpp)
target_link_libraries(test1)


# add excutable
set(EXECUTABLE_NAME "diffbot_node")
add_executable(${EXECUTABLE_NAME} src/diffbot_node.cpp)

target_link_libraries(${EXECUTABLE_NAME} test1)
ament_target_dependencies(diffbot_node 
  rclcpp std_msgs)



# install(TARGETS 
#   diffbot_node
#   DESTINATION lib/${PROJECT_NAME})



# install(TARGETS diffbot_base
#   DESTINATION lib/${PROJECT_NAME}
# )


## INSTALL
# install(
#   TARGETS ${PROJECT_NAME}
#   DESTINATION lib
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY src launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(test1)
ament_export_dependencies(rclcpp)
ament_package()
