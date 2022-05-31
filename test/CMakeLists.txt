# VIBes figures
option(WITH_VIBES "Generate VIBes figure" OFF)
if(WITH_VIBES)
message(STATUS "Generate VIBes figures")
endif()

# IPE figures
option(WITH_IPE "Generate IPE figure" OFF)
if(WITH_IPE)
  message(STATUS "Generate IPE figures")
endif()

# Tests folders
set(TEST_FILES 01-forward 02-backward 03-fb 03-detection_space 04-complex 05-video 06-presentation)

# Generating executables
foreach(file IN LISTS TEST_FILES)
    set(SOURCES ${file}/main.cpp ../src/scene.hpp ../src/sensor.hpp ../src/boat.hpp)
    add_executable(${file} ${SOURCES})
    target_include_directories(${file} PUBLIC ../src cxxopts)
    target_link_libraries(${file} ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} fmt::fmt ${IPEGENERATOR_LIBRARIES} wake_boat cxxopts ${CMAKE_THREAD_LIBS_INIT})
    if(WITH_VIBES)
        target_compile_definitions(${file} PUBLIC WITH_VIBES)
    endif()
    if(WITH_IPE)
        target_compile_definitions(${file} PUBLIC WITH_IPE)
    endif()
endforeach()