  
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
../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)

include_directories(${CMAKE_SOURCE_DIR}/faultdetection/behaviors
					)

target_link_libraries(epuck_swarm argos3plugin_${ARGOS_BUILD_FOR}_epuck) 



