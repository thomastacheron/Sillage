cmake_minimum_required(VERSION 3.5.1)
project(WakeBoat)

# Settings
set(CMAKE_CXX_STANDARD 11)
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
# Compile sources
################################################################################
add_executable(test src/main.cpp)
# target_include_directories(test ${IPEGENERATOR_INCLUDE_DIRS})
target_link_libraries(test ${IPEGENERATOR_LIBRARIES} ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES})
