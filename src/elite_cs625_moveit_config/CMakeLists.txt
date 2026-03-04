cmake_minimum_required(VERSION 3.22)
project(elite_cs625_moveit_config)

find_package(ament_cmake REQUIRED)

# Install the launch directory, excluding the setup assistant launch file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE
  )
endif()

# Install the config directory and all its contents
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the hidden .setup_assistant file
install(
    FILES .setup_assistant
    DESTINATION share/${PROJECT_NAME}
)

# ament_package() must be the last command
ament_package()
