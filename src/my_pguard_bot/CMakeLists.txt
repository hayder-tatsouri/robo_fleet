cmake_minimum_required(VERSION 3.8)
project(my_pguard_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch config description worlds rviz scripts foxglove maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/patrol_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME patrol_client
)

install(PROGRAMS
  scripts/save_camera_frame.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_camera_frame
)

install(PROGRAMS
  scripts/robo_fleet_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME robo_fleet_adapter
)

install(PROGRAMS
  scripts/gps_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_tf_publisher
)

ament_package()
