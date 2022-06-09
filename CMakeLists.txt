cmake_minimum_required(VERSION 3.20)
project(WakeBoat)

# Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3 -lpthread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -lpthread")

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
# Looking for IPE
################################################################################
include(FetchContent)
FetchContent_Declare(libipe
    URL https://github.com/otfried/ipe/releases/download/v7.2.24/ipe-7.2.24-src.tar.gz
    URL_HASH SHA256=feb879f24d1dce25910790770446d50b03e46ee2ebd7d28157b1705971644f6f
    DOWNLOAD_NO_EXTRACT true
)
FetchContent_MakeAvailable(libipe)

file(ARCHIVE_EXTRACT INPUT ${libipe_SOURCE_DIR}/ipe-7.2.24-src.tar.gz DESTINATION ${CMAKE_SOURCE_DIR}/extern/ipe)

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
# Looking for THREADS
################################################################################
find_package (Threads)

################################################################################
# Compile sources
################################################################################
add_library(wake_boat src/scene.cpp src/utils.cpp)
target_link_libraries(wake_boat ${CODAC_LIBRARIES} Ibex::ibex ${IPEGENERATOR_LIBRARIES} fmt::fmt)

add_executable(main src/main.cpp)
target_include_directories(main PUBLIC cxxopts)
target_link_libraries(main ${CODAC_LIBRARIES} Ibex::ibex ${IPEGENERATOR_LIBRARIES} fmt::fmt cxxopts ${CMAKE_THREAD_LIBS_INIT} wake_boat)

add_subdirectory(test)