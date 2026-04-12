cmake_minimum_required(VERSION 3.10)
project(EdgeSense LANGUAGES CXX)

# 1. Project-wide C++ Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compilation flags for professional safety
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Embed Git commit hash into the binary for version tracking
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Pass the hash into your C++ code as a macro definition
add_definitions(-DGIT_COMMIT_HASH="${GIT_HASH}")

# 2. Find Dependencies
find_package(Threads REQUIRED)

# 3. Define the Executable
set(BINARY_NAME EdgeSenseApp)

# 4. Collect Source Files
# We list them explicitly to maintain control over the build graph
set(PROJECT_SOURCES
    # Main Application Entry Point
    Src/main.cpp
    # Core Modules
    Src/Core/ThreadManager.cpp
    Src/Core/SensorManager.cpp
    Src/Core/CalibDataStore.cpp
    #  HAL Modules
    Src/HAL/I2cMaster.cpp
    # IPC Socket Server
    Src/IPC/SocketServer.cpp
    # Logger
    Src/Logger/Logger.cpp
    # Sensor Implementations
    Src/Sensors/SensorsRegistry.cpp
    Src/Sensors/LSM9DS1_ImuSensAccGyro.cpp
    Src/Sensors/LSM9DS1_ImuSensMag.cpp
    Src/Sensors/LPS25HB_EnvSens.cpp
    # Utility Functions
    Src/Utils/Utils.cpp
)

add_executable(${BINARY_NAME} ${PROJECT_SOURCES})

# 5. Handle Include Directories
# "PUBLIC" ensures that anyone linking against this target gets these headers
target_include_directories(${BINARY_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
)

# 6. Link Libraries
# Link POSIX Threads for your real-time scheduler
target_link_libraries(${BINARY_NAME} PRIVATE 
    Threads::Threads
    rt # Real-time library for clock_gettime
)

# 7. Install Rules (Optional, for 'make install')
install(TARGETS ${BINARY_NAME} DESTINATION bin)