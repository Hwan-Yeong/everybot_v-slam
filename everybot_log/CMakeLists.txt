cmake_minimum_required(VERSION 3.8)
project(everybot_log)

set(CMAKE_BUILD_TYPE Release) #set(CMAKE_BUILD_TYPE Debug) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add linker flags for memory protections
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro,-z,now")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro,-z,now")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# Include directories
include_directories(
  include  # Include your project's header directory
)

# Find libLOGNetwork headers (keep in include/everybot_log)
find_path(LIBNETWORK_INCLUDE_DIR
  NAMES libNetwork.h
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/everybot_log
  NO_DEFAULT_PATH
)

# Find libLOGNetwork.so (keep in include/everybot_log)
find_library(LIBNETWORK_LIBRARY 
  NAMES LOGNetwork libLOGNetwork
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/everybot_log
  NO_DEFAULT_PATH
)

if(NOT LIBNETWORK_INCLUDE_DIR OR NOT LIBNETWORK_LIBRARY)
  message(FATAL_ERROR "libLOGNetwork headers or library not found. Check the paths.")
endif()

# Executable
add_executable(everybot_logging src/everybot_logging.cpp)

# Link dependencies
ament_target_dependencies(everybot_logging rclcpp spdlog)

target_include_directories(everybot_logging PRIVATE ${LIBNETWORK_INCLUDE_DIR})
target_link_libraries(everybot_logging ${LIBNETWORK_LIBRARY} fmt::fmt spdlog::spdlog)

# Install targets
install(TARGETS
  everybot_logging
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/everybot_log/
  DESTINATION include/everybot_log
)

# Ensure libLOGNetwork.so is installed (keep in include/everybot_log)
install(FILES ${LIBNETWORK_LIBRARY}
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
