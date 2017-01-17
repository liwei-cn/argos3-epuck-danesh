  
add_executable(epuck_hom_swarm
epuck_hom_swarm/epuck_hom_swarm.h 
epuck_hom_swarm/epuck_hom_swarm.cpp 
epuck_hom_swarm/behavior.h 
epuck_hom_swarm/behavior.cpp 
epuck_hom_swarm/dispersebehavior.h 
epuck_hom_swarm/dispersebehavior.cpp 
epuck_hom_swarm/randomwalkbehavior.h 
epuck_hom_swarm/randomwalkbehavior.cpp 
epuck_hom_swarm/homingtofoodbeaconbehavior.h 
epuck_hom_swarm/homingtofoodbeaconbehavior.cpp 
epuck_hom_swarm/aggregatebehavior.h 
epuck_hom_swarm/aggregatebehavior.cpp 
epuck_hom_swarm/flockingbehavior.h 
epuck_hom_swarm/flockingbehavior.cpp 
epuck_hom_swarm/circlebehavior.h 
epuck_hom_swarm/circlebehavior.cpp 
../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)

target_link_libraries(epuck_hom_swarm argos3plugin_${ARGOS_BUILD_FOR}_epuck) 