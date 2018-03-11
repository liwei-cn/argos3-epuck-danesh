  
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
fault_detection_model/sensingandcommunication.h
fault_detection_model/sensingandcommunication.cpp
fault_detection_model/featurevectorsinrobotagent.h
fault_detection_model/featurevectorsinrobotagent.cpp
feature_vector/bayesianinferencefeaturevector.h
feature_vector/bayesianinferencefeaturevector.cpp
feature_vector/propriofeaturevector.h
feature_vector/propriofeaturevector.cpp
../plugins/robots/e-puck/real_robot/real_epuck_main.cpp)

include_directories(${CMAKE_SOURCE_DIR}/faultdetection/behaviors
					${CMAKE_SOURCE_DIR}/faultdetection/fault_detection_model
					${CMAKE_SOURCE_DIR}/faultdetection/feature_vector)

target_link_libraries(epuck_swarm argos3plugin_${ARGOS_BUILD_FOR}_epuck) 



