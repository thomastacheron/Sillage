cmake_minimum_required(VERSION 3.20)
project(WakeBoat)

# Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

################################################################################
# Looking for IBEX
################################################################################
find_package(IBEX REQUIRED)
ibex_init_common() # IBEX should have installed this function
message(STATUS "Found IBEX version ${IBEX_VERSION}")

################################################################################
# Looking for CODAC
################################################################################
find_package(CODAC REQUIRED)
message(STATUS "Found Codac version ${CODAC_VERSION}")
include_directories(${CODAC_INCLUDE_DIRS})
message(STATUS "Codac Include dirs ${CODAC_INCLUDE_DIRS}")

################################################################################
# Looking for IPE-Generator
################################################################################
find_package(ipegenerator REQUIRED)
message(STATUS "Found ipegenerator")
include_directories(${IPEGENERATOR_INCLUDE_DIRS})
set(LIBS ${IPEGENERATOR_LIBRARIES} ${LIBS})

################################################################################
# Looking for Eigen
################################################################################
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

################################################################################
# Looking for FMT
################################################################################
find_package(fmt REQUIRED)
include_directories(${fmt_INCLUDE_DIRS})

################################################################################
# Looking for CXXOPTS
################################################################################
add_subdirectory(extern/cxxopts)

################################################################################
# Compile sources
################################################################################
# add_executable(main src/main.cpp)
# target_link_libraries(main ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} fmt::fmt)

add_library(wake_boat src/scene.cpp)
target_link_libraries(wake_boat ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} ${IPEGENERATOR_LIBRARIES} fmt::fmt)

add_subdirectory(test)