cmake_minimum_required(VERSION 3.22)
project(simple_parking_maneuver)

find_package(rclpy)
find_package(std_msgs)
find_package(simple_drive_control)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        srv/ParkingManeuver.srv
        DEPENDENCIES std_msgs
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS src/parking_maneuver.py DESTINATION lib/${PROJECT_NAME})

ament_package()
