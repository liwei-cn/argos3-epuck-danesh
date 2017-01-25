#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_pseudo_range_and_bearing_sensor.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>

#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr
#include<fcntl.h>

#define TCP 0
#define UDP 1

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

        virtual int ConnectTrackingServer(int protocol);
        virtual bool ReceiveBuffer(UInt8* pun_buffer, size_t un_size);

        virtual unsigned RobotIdStrToInt()
        {
            std::string id = GetId();

            std::string::size_type sz;   // alias of size_t
            unsigned u_id = std::stoi(id, &sz);
            return u_id;
        }


    private:

        SInt32 left_wheel_speed;
        SInt32 right_wheel_speed;

        // Sensors
        CCI_EPuckProximitySensor* proximity_sensor;
        CCI_EPuckLightSensor* light_sensor;
        CCI_EPuckPseudoRangeAndBearingSensor* pseudo_rab_sensor;

        // Actuators
        CCI_EPuckWheelsActuator* wheels_actuator;
        CCI_EPuckBaseLEDsActuator* base_leds_actuator;

        int control_step;

        // Network socket communication
        int socket_desc; unsigned int un_connectionstatus;
        struct sockaddr_in server, client; 
        UInt8* pun_databuffer;
        unsigned int un_databuffersize, un_serveraddresslength;
        int in_recvpcktsize;
        char serveripaddress[100]; 
    };
};

#endif

