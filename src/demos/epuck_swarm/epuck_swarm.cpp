/**
 * @file <argos3-epuck/src/faultdetection/epuck_hom_swarm/epuck_hom_swarm.cpp>
 * This controller is meant to be used with the XML file: epuck_hom_swarm.argos
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

/****************************************/
/****************************************/
/* Include the controller definition */
#include "epuck_swarm.h"

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


/****************************************/
/****************************************/

CEPuckHomSwarm::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_AGGREGATION)
{
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ExperimentToRun::Init(TConfigurationNode& t_node)
{
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "ExperimentToRun Init function started " << std::endl;
#endif
    std::string str_behavior_transition_probability;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
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
            RobotId = (std::string(ipaddress).erase(0, 11));  //the ip address has 11 numbers
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
        m_strOutput = RobotId + "_" + swarmbehav + "_" + convert.str() + ".fvlog";


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
	else if (swarmbehav.compare("SWARM_OMEGA_ALGORITHM") == 0)
        SBehavior = SWARM_OMEGA_ALGORITHM;
    else
        THROW_ARGOSEXCEPTION("Invalid swarm behavior");

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
    b_damagedrobot(false),		//by default the robot is not faulty
    leftSpeed_prev(0.0f),
    rightSpeed_prev(0.0f),
    leftSpeed(0.0f),
    rightSpeed(0.0f),
    u_num_consequtivecollisions(0),
    b_randompositionrobot(true),
    m_bRobotSwitchedBehavior(false)
{

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
		m_pcLight 	 	  = GetSensor<CCI_EPuckLightSensor>("epuck_light");   //remember to add it; otherwise segment fault

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
	
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "Init function ended! " << std::endl;
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
	
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ControlStep()
{
	/*
		do some random movement and disperse the robots; sync with the tracking server
	*/
    if(b_randompositionrobot)
    {
#ifdef DEBUG_EXP_MESSAGES
        std::cout << "Waiting for the experiment to begin" << std::endl;
#endif

//	    m_pcLEDs->SwitchLED((int)(m_fInternalRobotTimer) % 8, true); // Turn one of the 8 base LEDs on
//        m_pcLEDs->SwitchLED((int)(m_fInternalRobotTimer - 1) % 8, false); // Turn previous base LED off

        m_pcLEDs->FrontLED((int)m_fInternalRobotTimer % 2 == 0);
        m_pcLEDs->BodyLED((int)m_fInternalRobotTimer % 2 == 1);
		
		m_fInternalRobotTimer+=1.0f;
		
        if(m_fInternalRobotTimer == 10) // Random positioning for 1s
        {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);

            b_randompositionrobot = false;
            m_fInternalRobotTimer = 0.0f;

			/*this makes sure the behavior won't be instantiated in every control step*/
			if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION               ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION                ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING                  ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING                    ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING_MOVING_BEACON      ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP                      ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION_DISPERSION    ||
			   m_sExpRun.SBehavior == ExperimentToRun::SWARM_OMEGA_ALGORITHM)
				 RunHomogeneousSwarmExperiment();
			
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

//    m_pcLEDs->SwitchLED((int)(m_fInternalRobotTimer) % 8, true); // Turn one of the 8 base LEDs on
//    m_pcLEDs->SwitchLED((int)(m_fInternalRobotTimer - 1) % 8, false); // Turn previous base LED off

    m_pcLEDs->FrontLED(false);
    m_pcLEDs->BodyLED(false);

	CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(), GetRABSensorReadings(), GetLightSensorReadings());  //no fault 

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
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)  //execute the behaviors
    {
        if (!bControlTaken)
        {
            
			if (m_sExpRun.SBehavior == ExperimentToRun::SWARM_OMEGA_ALGORITHM) 
			{
				bControlTaken = (*i)->TakeControl(m_fInternalRobotTimer);   //modified specifically for omega algorithm
				if (dynamic_cast<CEPuckOmegaAlgorithm*>(*i)->GetIlluminateStatus()) 
				{
					m_pcLEDs->BodyLED(true);
				}
				else 
				{
					m_pcLEDs->BodyLED(false);
				}
			}
			else 
			{
				bControlTaken = (*i)->TakeControl();
			}
			
			if (bControlTaken)
            {
#ifdef DEBUG_EXP_MESSAGES
                (*i)->PrintBehaviorIdentity();
#endif
                (*i)->Action(leftSpeed, rightSpeed);		//change the speed of the robot based on the sensor reading and controller
            }
        } else
            (*i)->Suppress();
    }

	printf("[IR]\t\t");
    for(CCI_EPuckProximitySensor::SReading reading : GetIRSensorReadings())
        printf("%.2f, ", reading.Value);
	printf("\n");


    //for(CCI_EPuckProximitySensor::SReading reading : GetIRSensorReadings(false, m_sExpRun.FBehavior))
    //    printf("%.2f, ", reading.Value);

	/*maybe because m_sSensoryData has already obstained the data from the tracking server, this read return empty data as the tracking server only send the data once??*/
/*
    CCI_EPuckPseudoRangeAndBearingSensor::TPackets rabsensor_readings = GetRABSensorReadings();
#ifdef DEBUG_EXP_MESSAGES
    std::cout << "Printing RAB Packets start " << std::endl;
    for(CCI_EPuckPseudoRangeAndBearingSensor::SReceivedPacket* m_pRABPacket : rabsensor_readings)
        printf("RobotId:%d, Range:%.2f, Bearing:%.2f\t", m_pRABPacket->RobotId, m_pRABPacket->Range, ToDegrees(m_pRABPacket->Bearing).GetValue());
    printf("\n");
    std::cout << "Printing RAB Packets end " << std::endl;
#endif
*/
/*
	printf("[LIGHT]\t\t");

    const CCI_EPuckLightSensor::TReadings& light_sensor_readings = m_pcLight->GetReadings();

    for(CCI_EPuckLightSensor::SReading reading: light_sensor_readings)
        printf("%.2f, ", reading.Value);

    printf("\n");
*/	
    //if(!(leftSpeed == leftSpeed_prev) && (rightSpeed == rightSpeed_prev))
    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s  //set the speed of the robot by writing into the actuactor SetLinearVelocity(leftSpeed, rightSpeed)

#ifdef DEBUG_EXP_MESSAGES
    std::cout << "LS:  " << leftSpeed << " RS:  " << rightSpeed << std::endl;
#endif

    //m_uRobotId = RobotIdStrToInt();

    /************************************************************************************/
    // Adding noise for the motor encoders
    leftSpeed_prev  += m_pcRNG->Uniform(CRange<Real>(-0.1f, 0.1f));
    rightSpeed_prev += m_pcRNG->Uniform(CRange<Real>(-0.1f, 0.1f));
    /************************************************************************************/

    m_fInternalRobotTimer+=1.0f;
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

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_OMEGA_ALGORITHM)
    {
		CEPuckOmegaAlgorithm* pcOmegaAlgorithm = new CEPuckOmegaAlgorithm(100.0f); //range threshold in cm //60.0
        m_vecBehaviors.push_back(pcOmegaAlgorithm);
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

	printf("id: %s\n", id.c_str());
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
