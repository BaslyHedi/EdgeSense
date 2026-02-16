cmake_minimum_required(VERSION 3.10)
project(EdgeSense LANGUAGES CXX)

# 1. Project-wide C++ Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add compilation flags for professional safety
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# 2. Find Dependencies
find_package(Threads REQUIRED)

# 3. Define the Executable
set(BINARY_NAME EdgeSenseApp)

# 4. Collect Source Files
# We list them explicitly to maintain control over the build graph
set(PROJECT_SOURCES
    Src/main.cpp
    Src/Core/Scheduler.cpp
    Src/Core/SensorManager.cpp
    Src/Hal/LinuxI2c.cpp
    Src/IPC/SocketServer.cpp
    Src/Logger/Logger.cpp
    Src/Sensors/ImuSensor.cpp
    Src/Sensors/EnvSensor.cpp
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