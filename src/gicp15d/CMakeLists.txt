cmake_minimum_required(VERSION 3.16)
project(gicp15d LANGUAGES CXX)

# Independent GICP-15D research module (Phase G1).
# Not part of the production a2rl_localization colcon build.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Local, already-present dependencies only. No download, no install.
set(SMALL_GICP_INCLUDE "" CACHE PATH "small_gicp upstream include directory")
if(NOT SMALL_GICP_INCLUDE)
  set(SMALL_GICP_INCLUDE
    "/home/ltc/flyagle/auto-eagle/src/a2rl_3rdparty/small_gicp_vendor/upstream/include")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(TBB QUIET)

if(NOT EXISTS "${SMALL_GICP_INCLUDE}/small_gicp/registration/registration.hpp")
  message(FATAL_ERROR
    "small_gicp headers not found at ${SMALL_GICP_INCLUDE}. "
    "Point -DSMALL_GICP_INCLUDE=<existing upstream/include> at the local checkout.")
endif()

add_library(gicp15d_core
  src/point_cloud_io.cpp
  src/local_covariance.cpp
  src/global_map_manager.cpp
  src/registration_diagnostics.cpp
  src/gicp_frontend.cpp
)
target_include_directories(gicp15d_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SMALL_GICP_INCLUDE}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(gicp15d_core PUBLIC OpenMP::OpenMP_CXX)
target_compile_options(gicp15d_core PRIVATE -Wall -Wextra -Wno-unused-parameter)

add_executable(gicp15d_frontend src/frontend_main.cpp)
target_link_libraries(gicp15d_frontend PRIVATE gicp15d_core)

add_executable(gicp15d_g0_service src/g0_service_main.cpp)
target_link_libraries(gicp15d_g0_service PRIVATE gicp15d_core)

add_executable(gicp15d_tests tests/run_tests.cpp)
target_link_libraries(gicp15d_tests PRIVATE gicp15d_core)

enable_testing()
add_test(NAME gicp15d_tests COMMAND gicp15d_tests)
