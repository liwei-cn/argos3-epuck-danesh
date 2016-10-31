#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>

namespace argos
{
    class CTestController : public CCI_Controller
    {
    public:

        CTestController();
        virtual ~CTestController();

        virtual void Init(TConfigurationNode& t_node);
        virtual void ControlStep();
        virtual void Reset() {};
        virtual void Destroy() {};

    private:

        SInt32 left_wheel_speed;
        SInt32 right_wheel_speed;

        // Sensors
        CCI_EPuckProximitySensor* proximity_sensor;
        CCI_EPuckLightSensor* light_sensor;

        // Actuators
        CCI_EPuckWheelsActuator* wheels_actuator;
        CCI_EPuckBaseLEDsActuator* base_leds_actuator;

        int control_step;
    };
};

#endif
