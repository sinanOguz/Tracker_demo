# File: CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(TrackingMonitor LANGUAGES C CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Locate OpenCV and Threads
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Include paths
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/third_party
)

# Build tinyfiledialogs as a static library
add_library(tinyfd STATIC
  ${CMAKE_SOURCE_DIR}/third_party/tinyfiledialogs.c
)
target_include_directories(tinyfd PUBLIC
  ${CMAKE_SOURCE_DIR}/third_party
)

# Gather all .cpp files under src/ automatically
file(GLOB TRACKER_SRC
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Main executable
add_executable(tracker_app
  ${TRACKER_SRC}
)


# Enable unused-code warnings for GNU/Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tracker_app PRIVATE
    -Wall
    -Wextra
    -Wunused-parameter
    -Wunused-variable
    -Wunused-function
  )
endif()

# Link libraries
target_link_libraries(tracker_app PRIVATE
  tinyfd                  # tinyfiledialogs
  ${OpenCV_LIBS}          # OpenCV modules
  Threads::Threads        # pthreads
)


# Debug output
message(STATUS "Building with sources:")
foreach(src IN LISTS TRACKER_SRC)
  message(STATUS "  ${src}")
endforeach()
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries:   ${OpenCV_LIBS}")
