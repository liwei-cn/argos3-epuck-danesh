#include <stdlib.h>

#include "test_controller.h"
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/vector2.h>

static std::ofstream file;

#define LOG_FILE_INIT(FILENAME) if (file) file.open (FILENAME);
#define LOG_FILE(SMTHG) if (file.is_open()) file << SMTHG;
#define LOG_FILE_CLOSE() if (file.is_open()) file.close();

using namespace argos;

CTestController::CTestController():
        control_step(0),
        left_wheel_speed(0),
        right_wheel_speed(0),
        wheels_actuator(NULL),
        proximity_sensor(NULL) {}

CTestController::~CTestController()
{

}

void CTestController::Init(TConfigurationNode& t_node)
{
    // Sensors
    proximity_sensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
    light_sensor = GetSensor<CCI_EPuckLightSensor>("epuck_light");

    // Actuators
    wheels_actuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    base_leds_actuator = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");
}

void CTestController::ControlStep()
{
    control_step++;

//    wheels_actuator->SetLinearVelocity(left_wheel_speed, right_wheel_speed);

    base_leds_actuator->SwitchLED(control_step % 8, true); // Turn one of the 8 base LEDs on
    base_leds_actuator->SwitchLED((control_step - 1) % 8, false); // Turn previous base LED off

    base_leds_actuator->FrontLED(control_step % 2 == 0);
    base_leds_actuator->BodyLED(control_step % 2 == 1);

    printf("[PROXIMITY]\t");

    const CCI_EPuckProximitySensor::TReadings& proximity_sensor_readings = proximity_sensor->GetReadings();

    for(CCI_EPuckProximitySensor::SReading reading : proximity_sensor_readings)
        printf("%.2f, ", reading.Value);
//        printf("%.0f degrees: %.2f, ", ToDegrees(reading.Angle).GetValue(), reading.Value);

    printf("\n");

    printf("[LIGHT]\t\t");

    const CCI_EPuckLightSensor::TReadings& light_sensor_readings = light_sensor->GetReadings();

    for(CCI_EPuckLightSensor::SReading reading: light_sensor_readings)
        printf("%.2f, ", reading.Value);

    printf("\n");

    printf("\n\n");

//    double velocity = CCI_EPuckWheelsActuator::MAX_VELOCITY_CM_SEC / 4;
//
//    CVector2 vector;
//
//    for(int i = 0; i < proximity_sensor_readings.size(); ++i)
//        vector += CVector2(proximity_sensor_readings[i].Value, proximity_sensor_readings[i].Angle);
//
//    vector /= proximity_sensor_readings.size();
//
//    printf("velocity: %f\n", velocity);
//    printf("length: %f, angle: %f\n", vector.Length(), ToDegrees(vector.Angle()));
//
//    if(vector.Length() > 50)
//    {
//        if(ToDegrees(vector.Angle()).GetValue() > 0.0f)
//            wheels_actuator->SetLinearVelocity(velocity, 0.0f);
//        else
//            wheels_actuator->SetLinearVelocity(0.0f, velocity);
//    }
//    else
//        wheels_actuator->SetLinearVelocity(velocity, velocity);
}

REGISTER_CONTROLLER(CTestController, "test_controller");
