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
        proximity_sensor(NULL),
        pseudo_rab_sensor(NULL) {}

CTestController::~CTestController()
{

}

void CTestController::Init(TConfigurationNode& t_node)
{
    // Sensors
    proximity_sensor = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
    light_sensor = GetSensor<CCI_EPuckLightSensor>("epuck_light");
    pseudo_rab_sensor = GetSensor<CCI_EPuckPseudoRangeAndBearingSensor>("epuck_pseudo_range_and_bearing");

    // Actuators
    wheels_actuator = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
    base_leds_actuator = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");

    /*un_databuffersize = 1024; //max size of data packet received from tracking server.
    pun_databuffer = new UInt8[un_databuffersize];
    un_connectionstatus = ConnectTrackingServer(UDP);

    un_serveraddresslength = sizeof(struct sockaddr_in);*/
 }

int CTestController::ConnectTrackingServer(int protocol)
{
    //Create socket
    if(protocol == TCP)
        socket_desc = socket(AF_INET , SOCK_STREAM , 0); // TCP socket
    else if(protocol == UDP)
        socket_desc = socket(AF_INET , SOCK_DGRAM, 0);     // UDP socket
    if (socket_desc == -1)
    {
        printf("Could not create socket");
        return 1;
    }

    bzero((char *) &server, sizeof(server));

    if(protocol == TCP)
    {
        server.sin_addr.s_addr = inet_addr("192.168.1.105"); //INADDR_ANY;
        server.sin_family = AF_INET;
        server.sin_port = htons(8888);

        //Connect to tracking server
        if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
        {
            printf("Could not connect to tracking server");
            return 1;
        }

        printf("Connected to tracking server \n");
    }

    if(protocol == UDP)
    {
        //Prepare the sockaddr_in structure for binding
        bzero((char *) &client, sizeof(client));
        client.sin_family = AF_INET;
        client.sin_addr.s_addr = INADDR_ANY;
        client.sin_port = htons(9000);

        //Bind. For UDP communicatio the client also has to bind to its port to receive packets from the server
        if(bind(socket_desc,(struct sockaddr *)&client , sizeof(client)) < 0)
        {
            std::cout << "bind failed" << std::endl;
            return 1;
        }
        std::cout << "bind done"  << std::endl;
    }


    if(fcntl(socket_desc, F_GETFL) & O_NONBLOCK)
    {
        printf("\nsocket is non-blocking");
    }
    return 0;
}

bool CTestController::ReceiveBuffer(UInt8* pun_buffer, size_t un_size) 
{
    ssize_t nReceived;
    while(un_size > 0)
    {
        nReceived = recv(socket_desc, pun_buffer, un_size, 0);
        if(nReceived < 0)
        {
            THROW_ARGOSEXCEPTION("Error receiving data: " << ::strerror(errno));
        }
        if(nReceived == 0)
            return false;
        un_size -= nReceived;
        pun_buffer += nReceived;

        printf("\nReceiving data");
    }
    printf("\nData received");
    return true;
}

void CTestController::ControlStep()
{
    control_step++;

    /*time_t* ptr_simple_time; time_t simple_time;
    ptr_simple_time = &simple_time; 
    simple_time = time(ptr_simple_time);
    struct tm* ptr_local_time;
    ptr_local_time = localtime(ptr_simple_time); 

    std::cout << ptr_local_time->tm_hour << " " << ptr_local_time->tm_min << " " << ptr_local_time->tm_sec << std::endl;*/


    /*if(un_connectionstatus==0)
    {
        bzero((UInt8*) pun_databuffer, un_databuffersize);
        //ReceiveBuffer(pun_databuffer, un_databuffersize); // not using this function as the packet size is not fixed
        in_recvpcktsize=0;
        //in_recvpcktsize = recv(socket_desc, pun_databuffer, un_databuffersize, 0);
        in_recvpcktsize = recvfrom(socket_desc, pun_databuffer, un_databuffersize, 0, (struct sockaddr *)&server, &un_serveraddresslength); // call will block if no data received. so should be made into a separate thread so as to not stall the rest of the control cycle if no data is received.

        if(in_recvpcktsize < 0)
        {
            //THROW_ARGOSEXCEPTION("Error receiving data: " << ::strerror(errno));
            printf("\nrecv error: %s\n", strerror(errno));
        }
        else
        {
            std::cout << "Received data packet of size " << in_recvpcktsize << " from " << inet_ntop(AF_INET, &(server.sin_addr), serveripaddress, INET_ADDRSTRLEN) << " port " << ntohs(server.sin_port) << std::endl;
        }

        if(in_recvpcktsize>0)
        {
            std::cout << std::endl << "Data packet contains ";
            for(int i=0;i<in_recvpcktsize;++i)
            {
                //std::cout << " " << pun_databuffer[i] << " ";
                printf(" %d ", pun_databuffer[i]);
            }
            std::cout << std::endl;
        }
    }*/


//    wheels_actuator->SetLinearVelocity(left_wheel_speed, right_wheel_speed);

    std::cout << "[Robot Id] " << this->GetId() << " => " << RobotIdStrToInt() << std::endl;

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

    printf("[PSEUDO RAB]\t");

    const CCI_EPuckPseudoRangeAndBearingSensor::TPackets& pseudo_rab_sensor_readings = pseudo_rab_sensor->GetPackets();

    for(CCI_EPuckPseudoRangeAndBearingSensor::SReceivedPacket* m_pRABPacket : pseudo_rab_sensor_readings)
        printf("RobotId:%d, Range:%.2f, Bearing:%.2f\t", m_pRABPacket->RobotId, m_pRABPacket->Range, m_pRABPacket->Bearing.GetValue());
    printf("\n");

    printf("\n\n");


    double velocity = CCI_EPuckWheelsActuator::MAX_VELOCITY_CM_SEC / 3.0f; // CCI_EPuckWheelsActuator::MAX_VELOCITY_CM_SEC = 30cm/s

    //Stage 1 -- robot moves straight for 50cm
    if(control_step < 50)
    {
        std::cout << "Stage 1 -- moving straight 50cm " << std::endl;
        wheels_actuator->SetLinearVelocity(velocity , velocity);
    }
    //Stage 2 -- robot turns clockwise pi radians
    else if (control_step > 50 && control_step <= 58)
    {
        std::cout << "Stage 2 -- robot turns clockwise pi radians " << std::endl;
        // With motor speeds at +10cm/s (left) and -10cm/s (right), the robot turns 3.77358490566 rad/s. So turning pi radians would take 0.8325220532s = 8.3 steps
        wheels_actuator->SetLinearVelocity(velocity, -velocity);
    }
    //Stage 3 -- robot turns counter-clockwise 2pi radians
    else if (control_step > 58 && control_step <= 74)
    {
        std::cout << "Stage 3 -- robot turns counter-clockwise 2pi radians " << std::endl;
        // With motor speeds at +10cm/s (left) and -10cm/s (right), the robot turns 3.77358490566 rad/s. So turning 2pi radians would take 1.6650441064s = 16.65 steps
        wheels_actuator->SetLinearVelocity(-velocity, velocity);
    }
    //Stage 4 -- robot again moves straight 50cm
    else if (control_step > 74 && control_step <= (74+50))
    {
        std::cout << "Stage 4 -- robot now returning 50cm" << std::endl;
        // With motor speeds at +10cm/s (left) and -10cm/s (right), the robot turns 3.77358490566 rad/s. So turning 2pi radians would take 1.6650441064s = 16.65 steps
        wheels_actuator->SetLinearVelocity(velocity, velocity);
    }
    else
    {
        std::cout << "Test complete" << std::endl;
        wheels_actuator->SetLinearVelocity(0.0f, 0.0f);
    }
}

REGISTER_CONTROLLER(CTestController, "test_controller");
