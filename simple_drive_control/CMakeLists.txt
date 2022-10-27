cmake_minimum_required(VERSION 3.22)
project(simple_drive_control)

find_package(rclpy)
find_package(std_msgs)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        srv/DrivingManeuver.srv
        DEPENDENCIES std_msgs
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS src/drive_control.py DESTINATION lib/${PROJECT_NAME})

ament_package()
