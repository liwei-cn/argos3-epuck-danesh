/**
 * @file <argos3-epuck/src/faultdetection/epuck_hom_swarm/epuck_hom_swarm.cpp>
 * This controller is meant to be used with the XML file: epuck_hom_swarm.argos
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

/****************************************/
/****************************************/
/* Include the controller definition */
#include "epuck_hom_swarm.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/plugins/robots/e-puck/real_robot/real_epuck.h>

/* Network communication with tracking server to sync robots*/
#include<stdio.h>
#include<string.h>    //strlen
#include<sys/socket.h>
#include<arpa/inet.h> //inet_addr

#include <netdb.h>
#include <ifaddrs.h>
#include <stdlib.h>

#include <sstream>
#include <string>

/****************************************/
/****************************************/

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

CProprioceptiveFeatureVector::RobotData CProprioceptiveFeatureVector::m_sRobotData;
CBayesianInferenceFeatureVector::RobotData CBayesianInferenceFeatureVector::m_sRobotData;

/****************************************/
/****************************************/

CEPuckHomSwarm::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_AGGREGATION),
    FBehavior(FAULT_NONE),
    id_FaultyRobotInSwarm("-1")
{
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ExperimentToRun::Init(TConfigurationNode& t_node)
{
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "ExperimentToRun Init function started " << std::endl;
#endif
    std::string errorbehav; std::string str_behavior_transition_probability;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobotInSwarm);
        GetNodeAttribute(t_node, "output_filename", m_strOutput); // not used anymore

        GetNodeAttribute(t_node, "behavior_transition_probability", str_behavior_transition_probability);
        behavior_transition_probability = strtold(str_behavior_transition_probability.c_str(), NULL);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);


    /***************************************************/
    struct ifaddrs *ifaddr, *ifa;
    int s;
    char ipaddress[NI_MAXHOST];

    if (getifaddrs(&ifaddr) == -1)
        THROW_ARGOSEXCEPTION("Error getifaddrs");

    bool id_found = false;
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;

        s=getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), ipaddress, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if((s==0) && (strcmp(ifa->ifa_name, "wlan0") == 0) && (ifa->ifa_addr->sa_family==AF_INET))
        {
            RobotId = (std::string(ipaddress).erase(0, 10));
            id_found = true;
            break;
        }
    }

    if(!id_found)
    {
        THROW_ARGOSEXCEPTION("Failed to get robot id from ip address used to name log files");
    }

    freeifaddrs(ifaddr);
    /***************************************************/


    std::ostringstream convert;
    convert << CRealEPuck::GetInstance().GetRandomSeed();

    if (swarmbehav.compare("SWARM_AGGREGATION_DISPERSION") == 0)
    {
        m_strOutput = RobotId + "_" + swarmbehav + "_" + str_behavior_transition_probability + "_" + convert.str() + ".fvlog";
    }
    else
        m_strOutput = RobotId + "_" + swarmbehav + "_" + errorbehav + "_" + convert.str() + ".fvlog";


    /* Open the file, erasing its contents */
    m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
    //m_cOutput << "TimeStamp\tInternalCounter\tRobot_Id\tProprioceptiveFV\tObserving_RobotIds\tObserving_RobotFVs\n" << std::endl;




    if (swarmbehav.compare("SWARM_AGGREGATION") == 0)
        SBehavior = SWARM_AGGREGATION;
    else if (swarmbehav.compare("SWARM_DISPERSION") == 0)
        SBehavior = SWARM_DISPERSION;
    else if (swarmbehav.compare("SWARM_FLOCKING") == 0)
        SBehavior = SWARM_FLOCKING;
    else if (swarmbehav.compare("SWARM_HOMING") == 0)
        SBehavior = SWARM_HOMING;
    else if (swarmbehav.compare("SWARM_HOMING_MOVING_BEACON") == 0)
        SBehavior = SWARM_HOMING_MOVING_BEACON;
    else if (swarmbehav.compare("SWARM_STOP") == 0)
        SBehavior = SWARM_STOP;
    else if (swarmbehav.compare("SWARM_AGGREGATION_DISPERSION") == 0)
    {
        SBehavior = SWARM_AGGREGATION_DISPERSION;
        assert(behavior_transition_probability >= 0.0f && behavior_transition_probability <= 1.0f);
    }
    else
        THROW_ARGOSEXCEPTION("Invalid swarm behavior");


    if (errorbehav.compare("FAULT_NONE") == 0)
        FBehavior = FAULT_NONE;
    else if  (errorbehav.compare("FAULT_STRAIGHTLINE") == 0)
        FBehavior = FAULT_STRAIGHTLINE;
    else if  (errorbehav.compare("FAULT_RANDOMWALK") == 0)
        FBehavior = FAULT_RANDOMWALK;
    else if  (errorbehav.compare("FAULT_CIRCLE") == 0)
        FBehavior = FAULT_CIRCLE;
    else if  (errorbehav.compare("FAULT_STOP") == 0)
        FBehavior = FAULT_STOP;

    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMIN") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMIN;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMAX") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMAX;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETRANDOM;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETOFFSET") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETOFFSET;


    else if  (errorbehav.compare("FAULT_RABSENSOR_SETOFFSET") == 0)
        FBehavior = FAULT_RABSENSOR_SETOFFSET;

    else if  (errorbehav.compare("FAULT_ACTUATOR_LWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_LWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_RWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_RWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_BWHEELS_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_BWHEELS_SETZERO;

    else
        THROW_ARGOSEXCEPTION("Invalid fault behavior");
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "ExperimentToRun Init function ended" << std::endl;
#endif
}

/****************************************/
/****************************************/

CEPuckHomSwarm::CEPuckHomSwarm() :
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcRNG(CRandom::CreateRNG("argos")),
    m_pcRNG_FVs(CRandom::CreateRNG("argos")),
    b_damagedrobot(false),
    leftSpeed_prev(0.0f),
    rightSpeed_prev(0.0f),
    leftSpeed(0.0f),
    rightSpeed(0.0f),
    u_num_consequtivecollisions(0),
    b_randompositionrobot(true),
    m_bRobotSwitchedBehavior(false)
{
    listFVsSensed.clear();
    listMapFVsToRobotIds.clear();
    listMapFVsToRobotIds_relay.clear();

    m_uRobotFV = 9999; // for debugging purposes

    m_fRobotTimerAtStart = 0.0f;
    m_fInternalRobotTimer = m_fRobotTimerAtStart;

    m_sExpRun.RobotId = GetId();
}

/****************************************/
/****************************************/

CEPuckHomSwarm::~CEPuckHomSwarm()
{
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "Destroying CEPuckHomSwarm controller " << std::endl;
    std::cout << "Finished destroying CEPuckHomSwarm controller " << std::endl;
#endif
    m_sExpRun.m_cOutput.close();
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Init(TConfigurationNode& t_node)
{
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "Init function started " << std::endl;
#endif
    try
    {
        /*
       * Initialize sensors/actuators
       */
        m_pcProximity     = GetSensor  <CCI_EPuckProximitySensor>("epuck_proximity");
        m_pcRABS          = GetSensor  <CCI_EPuckPseudoRangeAndBearingSensor>("epuck_pseudo_range_and_bearing");

        m_pcWheels        = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
        m_pcLEDs          = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");

        /*
       * Parse XML parameters
       */
        /* Experiment to run */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck hom_swarm controller for robot \"" << GetId() << "\"", ex);


    Reset();

    // CCI_EPuckWheelsActuator::MAX_VELOCITY_CM_SEC / 3.0f = 10 cm/s
    m_sRobotDetails.SetKinematicDetails(CCI_EPuckWheelsActuator::MAX_VELOCITY_CM_SEC / 3.0f, CCI_EPuckWheelsActuator::MAX_VELOCITY_CM_SEC / 3.0f);
    //m_sRobotDetails.SetKinematicDetails(2.5f, 2.5f);

    CopyRobotDetails(m_sRobotDetails);


    m_pFlockingBehavior = new CFlockingBehavior(m_sRobotDetails.iterations_per_second * 1.0f); // 5.0f

    //if(this->GetId().compare("ep"+m_sExpRun.id_FaultyRobotInSwarm) == 0)
    if(this->GetId().compare(m_sExpRun.id_FaultyRobotInSwarm) == 0)
        b_damagedrobot = true;

#ifdef DEBUG_EXP_MESSAGES
    std::cout << "Init function ended " << std::endl;
#endif
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::CopyRobotDetails(RobotDetails& robdetails)
{
    CBehavior::m_sRobotData.MaxSpeed                    = robdetails.MaxLinearSpeed * robdetails.iterations_per_second; // max speed in cm/s to control behavior
    CBehavior::m_sRobotData.iterations_per_second       = robdetails.iterations_per_second;
    CBehavior::m_sRobotData.seconds_per_iterations      = 1.0f / robdetails.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE    = robdetails.HALF_INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE         = robdetails.INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.WHEEL_RADIUS                = robdetails.WHEEL_RADIUS;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = robdetails.m_cNoTurnOnAngleThreshold;
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = robdetails.m_cSoftTurnOnAngleThreshold;


    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CProprioceptiveFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;


    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CBayesianInferenceFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CBayesianInferenceFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;

    CBayesianInferenceFeatureVector::m_sRobotData.SetLengthOdometryTimeWindows();
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ControlStep()
{
    if(b_randompositionrobot)
    {
#ifdef DEBUG_EXP_MESSAGES
        std::cout << "Positioning the robot to new random position by running dispersion behavior" << std::endl;
#endif

        m_vecBehaviors.clear();
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f))); //new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)))
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f); //0.0017f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(false, m_sExpRun.FBehavior), GetRABSensorReadings(false, m_sExpRun.FBehavior));

        leftSpeed_prev = leftSpeed; rightSpeed_prev = rightSpeed;
        leftSpeed = 0.0; rightSpeed = 0.0f;
        bool bControlTaken = false;
        for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
        {
            if (!bControlTaken)
            {
                bControlTaken = (*i)->TakeControl();
                if (bControlTaken)
                {
#ifdef DEBUG_EXP_MESSAGES
                    (*i)->PrintBehaviorIdentity();
#endif
                    (*i)->Action(leftSpeed, rightSpeed);
                }
            } else
                (*i)->Suppress();
        }
        //if(!(leftSpeed == leftSpeed_prev) && (rightSpeed == rightSpeed_prev))
        m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s
        m_fInternalRobotTimer+=1.0f;


        if(m_fInternalRobotTimer == 299) // Random positioning for 10s
        {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        }

        if(m_fInternalRobotTimer == 300) // Random positioning for 10s
        {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);

            b_randompositionrobot = false;
            m_fInternalRobotTimer = 0.0f;

            /**
                Send message to tracking server -- exp port
                On receiving reply, start expt.
            */

            int socket_desc;
            struct sockaddr_in trackingserver;

            //Create socket
            socket_desc = socket(AF_INET , SOCK_STREAM , 0);
            if (socket_desc == -1)
            {
                printf("Could not create socket");
            }

            trackingserver.sin_addr.s_addr = inet_addr(TRACKING_SERVER_IPADDRESS);
            trackingserver.sin_family = AF_INET;
            trackingserver.sin_port = htons(ROBOTS_SYNC_PORT);

            //Connect to remote trackingserver
            if (connect(socket_desc , (struct sockaddr *)&trackingserver , sizeof(trackingserver)) < 0)
                THROW_ARGOSEXCEPTION("Connection error. Could not connect to tracking server for robot sync. IP address may have changed. Please check and change TRACKING_SERVER_IPADDRESS variable in epuck_hom_swarm.h"  << ::strerror(errno));

            std::cout << "Connected to tracking server " << std::endl;

            //Send data
            char server_message[] = "Requesting experiment start";
            if( send(socket_desc , server_message , strlen(server_message) , 0) < 0)
                THROW_ARGOSEXCEPTION("Sending message to tracking server for robot sync. has failed");

            std::cout << "Data sent to tracking server for robot sync." << std::endl;

            char server_reply[2000];
            //Receive a reply from the server
            if(recv(socket_desc, server_reply , 2000 , 0) < 0)
                THROW_ARGOSEXCEPTION("Recv reply from tracking server for robot sync. has failed");

            std::cout << "Reply received from tracking server for robot sync.\n" << std::endl;

            puts(server_reply);

            // Can now start experiment
#ifdef DEBUG_EXP_MESSAGES
            std::cout << "STARTING EXPERIMENT..." << std::endl;
#endif
        }
        return;
    }

#ifdef DEBUG_EXP_MESSAGES
    std::cout << std::endl << std::endl << "Control-step " << m_fInternalRobotTimer << " start " << std::endl;
#endif

    m_pcLEDs->SwitchLED((int)(m_fInternalRobotTimer) % 8, true); // Turn one of the 8 base LEDs on
    m_pcLEDs->SwitchLED((int)(m_fInternalRobotTimer - 1) % 8, false); // Turn previous base LED off

    m_pcLEDs->FrontLED((int)m_fInternalRobotTimer % 2 == 0);
    m_pcLEDs->BodyLED((int)m_fInternalRobotTimer % 2 == 1);


    bool b_RunningGeneralFaults(false);
    if(b_damagedrobot && (m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP))
    {
        b_RunningGeneralFaults = true;
        RunGeneralFaults();
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION               ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION                ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING                  ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING                    ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING_MOVING_BEACON      ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP                      ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION_DISPERSION)
        RunHomogeneousSwarmExperiment();


    if(!b_damagedrobot || b_RunningGeneralFaults || m_sExpRun.FBehavior == ExperimentToRun::FAULT_NONE)
        CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    else
    {
        //m_pcLEDs->SetAllColors(CColor::RED);

        if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMIN)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMAX)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETRANDOM)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));

        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));

        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
    }

    /*For flocking behavior - to compute relative velocity*/
    //CBehavior::m_sSensoryData.SetWheelSpeedsFromEncoders(m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
    CBehavior::m_sSensoryData.SetWheelSpeedsFromEncoders(leftSpeed_prev, rightSpeed_prev); // the encoders will give the speed of the wheels set at the previous control-cycle

    /**
     * The robot has to continually track the velocity of its neighbours - since this is done over a period of time. It can't wait until the flocking behavior is activated to start tracking neighbours.
     * However as the flocking behavior is not to be tested, we disable this continious tracking
     */
//#ifdef DEBUG_EXP_MESSAGES
//    std::cout << "Flocking behavior simulation step start " << std::endl;
//#endif

//    m_pFlockingBehavior->SimulationStep();
//#ifdef DEBUG_EXP_MESSAGES
//    std::cout << "Flocking behavior simulation step end " << std::endl;
//#endif

    leftSpeed_prev = leftSpeed; rightSpeed_prev = rightSpeed;
    leftSpeed = 0.0; rightSpeed = 0.0f;
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        if (!bControlTaken)
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
            {
                /*if(b_damagedrobot)
                      (*i)->PrintBehaviorIdentity();*/
#ifdef DEBUG_EXP_MESSAGES
                (*i)->PrintBehaviorIdentity();
#endif
                (*i)->Action(leftSpeed, rightSpeed);
            }
        } else
            (*i)->Suppress();
    }


    if(GetIRSensorReadings(false, m_sExpRun.FBehavior)[0].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[1].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[2].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[3].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[4].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[5].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[6].Value > 0.4f ||
       GetIRSensorReadings(false, m_sExpRun.FBehavior)[7].Value > 0.4f)
        u_num_consequtivecollisions++;
    else
        u_num_consequtivecollisions = 0u;

    for(CCI_EPuckProximitySensor::SReading reading : GetIRSensorReadings(false, m_sExpRun.FBehavior))
        printf("%.2f, ", reading.Value);


    // if the robot is colliding with the wall other robot for more than 5s, we reduce its speed by half
    /* this will be harder to detect when we add noise on the IR sensors. Be wary of that. So using the noiseless variant of the IR sensors for this detection*/
    if((Real)u_num_consequtivecollisions > (m_sRobotDetails.iterations_per_second * 1.0f))
    {
#ifdef DEBUG_EXP_MESSAGES
        std::cout << std::endl << "[Pseudo Motor Encoder Implementation] Prolonged collisions detected. Reducing motor speeds by 50%" << std::endl;
#endif

        leftSpeed  = leftSpeed/2.0f;
        rightSpeed = rightSpeed/2.0f;
    }


    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        leftSpeed  = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        rightSpeed = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
    {
        leftSpeed = 0.0f;
        rightSpeed = 0.0f;
    }

    //for(CCI_EPuckProximitySensor::SReading reading : GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior))
    //    printf("%.2f, ", reading.Value);

    CCI_EPuckPseudoRangeAndBearingSensor::TPackets rabsensor_readings = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "Printing RAB Packets start " << std::endl;
    for(CCI_EPuckPseudoRangeAndBearingSensor::SReceivedPacket* m_pRABPacket : rabsensor_readings)
        printf("RobotId:%d, Range:%.2f, Bearing:%.2f\t", m_pRABPacket->RobotId, m_pRABPacket->Range, ToDegrees(m_pRABPacket->Bearing).GetValue());
    printf("\n");
    std::cout << "Printing RAB Packets end " << std::endl;
#endif

    //if(!(leftSpeed == leftSpeed_prev) && (rightSpeed == rightSpeed_prev))
    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s

#ifdef DEBUG_EXP_MESSAGES
    std::cout << "LS:  " << leftSpeed << " RS:  " << rightSpeed << std::endl;
#endif

    m_uRobotId = RobotIdStrToInt();

    /************************************************************************************/
    // Adding noise for the motor encoders
    leftSpeed_prev  += m_pcRNG->Uniform(CRange<Real>(-0.1f, 0.1f));
    rightSpeed_prev += m_pcRNG->Uniform(CRange<Real>(-0.1f, 0.1f));
    /************************************************************************************/

#ifdef DEBUG_EXP_MESSAGES
    std::cout << "SenseCommunicateDetect-step start " << std::endl;
#endif
    SenseCommunicateDetect(m_sExpRun.m_cOutput,
                           RobotIdStrToInt(), leftSpeed_prev, rightSpeed_prev,
                           m_fInternalRobotTimer, rabsensor_readings,
                           listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed,
                           m_cProprioceptiveFeatureVector, m_cBayesianInferredFeatureVector, m_pcRNG_FVs, m_uRobotFV, m_sExpRun.swarmbehav, beaconrobots_ids);

#ifdef DEBUG_EXP_MESSAGES
    std::cout << "SenseCommunicateDetect-step end " << std::endl;
    std::cout << "Control-step " << m_fInternalRobotTimer << " end " << std::endl << std::endl;
#endif

    m_fInternalRobotTimer+=1.0f;
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunGeneralFaults()
{
    //m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
        m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);  // 0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        m_vecBehaviors.push_back(pcCircleBehavior);
    }

    else //m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP
    {}
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunHomogeneousSwarmExperiment()
{
    m_vecBehaviors.clear();

    if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f))); // 0.1f reflects a distance of about 4.5cm. Reduced to 0.02 for physical robots
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(100.0f); //range threshold in cm //60.0
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //m_pcLEDs->SetAllColors(CColor::GREEN);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f))); //new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)))
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //m_pcLEDs->SetAllColors(CColor::RED);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        m_vecBehaviors.push_back(m_pFlockingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING)
    {
        UInt8 BeaconRobotId = 202;
        if(m_uRobotId == BeaconRobotId)
        {
            // ep202 is the beacon robot
        }
        else
        {
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcDisperseBehavior);

            Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BeaconRobotId, MAX_BEACON_SIGNAL_RANGE);
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }
    }
    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING_MOVING_BEACON)
    {
        UInt8 BeaconRobotId = 202;
        if(m_uRobotId == BeaconRobotId)
        {
            // ep202 is the beacon robot
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f))); //new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)))
            m_vecBehaviors.push_back(pcDisperseBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }
        else
        {
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcDisperseBehavior);

            Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BeaconRobotId, MAX_BEACON_SIGNAL_RANGE);
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP)
    {
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION_DISPERSION)
    {
        // check to switch behavior -- (expected waiting time 1/p steps). As the check is made every 100 steps, it becomes 100/p steps
        // p = 1; instantaneous switch
        // p =.1; 1000steps = 10s
        // p =.05; 2000 steps = 20s
         if(m_fInternalRobotTimer >= 1500 && !m_bRobotSwitchedBehavior && ((unsigned)m_fInternalRobotTimer)%100 == 0)
         {
             if(m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f)) <= m_sExpRun.behavior_transition_probability)
             {
                 m_bRobotSwitchedBehavior = true;
                 m_sExpRun.m_cOutput << "Switch made to Dispersion " << std::endl;
             }

         }

        if(m_fInternalRobotTimer < 1500.0f || !m_bRobotSwitchedBehavior)
        {
            // Peform Aggregation
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f))); // 0.1f reflects a distance of about 4.5cm. Reduced to 0.02 for physical robots
            m_vecBehaviors.push_back(pcDisperseBehavior);

            CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(100.0f); //range threshold in cm //60.0
            m_vecBehaviors.push_back(pcAggregateBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);

        }
        else
        {
            // switch to dispersion
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.02f, ToRadians(CDegrees(5.0f))); //new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)))
            m_vecBehaviors.push_back(pcDisperseBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }

    }

}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Reset()
{
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::RobotIdStrToInt()
{
    std::string id = GetId();

    std::string::size_type sz;   // alias of size_t
    unsigned u_id = std::stoi(id, &sz);
    return u_id;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckHomSwarm, "epuck_homswarm_controller");
