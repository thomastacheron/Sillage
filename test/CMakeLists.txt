# Tests folders
set(TEST_FILES 01-forward 02-backward 03-detection_space 04-complex 05-video)

# Generating executables
foreach(file IN LISTS TEST_FILES)
    set(SOURCES ${file}/main.cpp ../src/scene.hpp ../src/sensor.hpp ../src/boat.hpp)
    add_executable(${file} ${SOURCES})
    target_include_directories(${file} PUBLIC ../src)
    target_link_libraries(${file} ${CODAC_LIBRARIES} Ibex::ibex ${CODAC_LIBRARIES} fmt::fmt ${IPEGENERATOR_LIBRARIES})
endforeach()