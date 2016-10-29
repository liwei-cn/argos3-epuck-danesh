#include <stdlib.h>

#include "test_controller.h"
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <argos3/core/utility/logging/argos_log.h>

static std::ofstream file;

#define LOG_FILE_INIT(FILENAME) if (file) file.open (FILENAME);
#define LOG_FILE(SMTHG) if (file.is_open()) file << SMTHG;
#define LOG_FILE_CLOSE() if (file.is_open()) file.close();

using namespace argos;

CTestController::CTestController() :
        left_wheel_speed(0),
        right_wheel_speed(0),
        wheels_actuator(NULL),
        proximity_sensor(NULL) {}

CTestController::~CTestController()
{

}

void CTestController::Init(TConfigurationNode& t_node)
{
    printf("Initialising controller\n");

    wheels_actuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    proximity_sensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");

    printf("Finished initialising controller\n");
}

void CTestController::ControlStep()
{
//    wheels_actuator->SetLinearVelocity(left_wheel_speed, right_wheel_speed);

    printf("[PROXIMITY]\t");

    for(int i = 0; i < 8; i++)
        printf("(%6.2f - %6.2f) ", proximity_sensor->GetReading(i).Value, ToDegrees(proximity_sensor->GetReading(i).Angle).GetValue());

    printf("\n");
}

REGISTER_CONTROLLER(CTestController, "test_controller");
