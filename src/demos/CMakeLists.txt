  
add_executable(epuck_swarm
epuck_swarm/epuck_swarm.h 
epuck_swarm/epuck_swarm.cpp 
behaviors/behavior.h 
behaviors/behavior.cpp 
behaviors/dispersebehavior.h 
behaviors/dispersebehavior.cpp 
behaviors/randomwalkbehavior.h 
behaviors/randomwalkbehavior.cpp 
behaviors/homingtofoodbeaconbehavior.h 
behaviors/homingtofoodbeaconbehavior.cpp 
behaviors/aggregatebehavior.h 
behaviors/aggregatebehavior.cpp 
behaviors/flockingbehavior.h 
behaviors/flockingbehavior.cpp 
behaviors/circlebehavior.h 
behaviors/circlebehavior.cpp 
behaviors/omega_algorithm.h 
behaviors/omega_algorithm.cpp 
../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)

include_directories(${CMAKE_SOURCE_DIR}/demos/behaviors
					)

target_link_libraries(epuck_swarm argos3plugin_${ARGOS_BUILD_FOR}_epuck) 

message("Cmake Source Dir: " ${CMAKE_SOURCE_DIR})
message("ARGoS Build For: " ${ARGOS_BUILD_FOR})



