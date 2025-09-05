

#................................................


cmake_minimum_required(VERSION 3.5)
project(main_controller)
find_package(ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED) ##

find_package(rclpy REQUIRED)          # |
find_package(geometry_msgs REQUIRED)  # |  these are the three dependencies in </depend>
find_package(std_msgs REQUIRED)       # |

find_package(sensor_msgs REQUIRED) 
find_package(nav_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/custom_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_copyright_FOUND True)
  set(ament_cmake_cpplint_FOUND True)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()