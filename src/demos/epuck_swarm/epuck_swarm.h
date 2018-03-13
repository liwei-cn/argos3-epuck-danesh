/**
 * @file <argos3-epuck/src/faultdetection/epuck_hom_swarm/epuck_hom_swarm.h>
 * This controller is meant to be used with the XML file: epuck_hom_swarm.argos
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

#ifndef EPUCK_HOMSWARM_H
#define EPUCK_HOMSWARM_H

#define DEBUG_EXP_MESSAGES
#define TRACKING_SERVER_IPADDRESS "144.32.145.252"
#define ROBOTS_SYNC_PORT 10021

/****************************************/
/****************************************/
/* ARGoS headers */

/* Deubg message flags*/
#include <argos3/plugins/robots/e-puck/real_robot/real_epuck_debugmessages.h>

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>

/* Definition of the e-puck proximity sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>

/* Definition of the pseudo range and bearing sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_pseudo_range_and_bearing_sensor.h>

/* Definition of the e-puck light sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>

/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/****************************************/
/****************************************/
/* Definitions for behaviors used to control robot */

#include "behavior.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "randomwalkbehavior.h"
#include "homingtofoodbeaconbehavior.h"
#include "circlebehavior.h"
#include "flockingbehavior.h"
#include "omega_algorithm.h"

/****************************************/
/****************************************/

using namespace argos;


class CEPuckHomSwarm : public CCI_Controller
{
public:

    struct ExperimentToRun
    {
        /* The type of experiment to run */
        enum SwarmBehavior
        {
            SWARM_AGGREGATION = 0,
            SWARM_DISPERSION,
            SWARM_HOMING,
            SWARM_HOMING_MOVING_BEACON,
            SWARM_FLOCKING,
            SWARM_STOP,
            SWARM_NONE,
            SWARM_AGGREGATION_DISPERSION,
			SWARM_OMEGA_ALGORITHM
        };
        enum SwarmBehavior SBehavior;

        std::string swarmbehav;        

        std::string m_strOutput;
        std::ofstream m_cOutput;

        Real behavior_transition_probability;

        std::string RobotId;

        ExperimentToRun();
        void Init(TConfigurationNode& t_node);
    };

    struct RobotDetails
    {
        Real iterations_per_second; /* controlcycles run per second*/
        Real seconds_per_iterations;
        Real HALF_INTERWHEEL_DISTANCE;  // in m
        Real INTERWHEEL_DISTANCE;  // in m
        Real WHEEL_RADIUS;  // in m

        CRadians m_cNoTurnOnAngleThreshold; CRadians m_cSoftTurnOnAngleThreshold;

        Real MaxLinearSpeed; //cm/ (controlcycle)
        Real MaxLinearAcceleration; //cm/controlcycle/controlcycle

        Real MaxAngularSpeed; //rad/controlcycle
        Real MaxAngularAcceleration; //rad/controlcycle/controlcycle;

        RobotDetails()
        {
            iterations_per_second  = 10.0f; /*10 ticks per second so dt=0.01s. i.e., the controlcycle is run 10 times per second*/
            seconds_per_iterations = 1.0f / iterations_per_second;
            HALF_INTERWHEEL_DISTANCE = 0.053f * 0.5f;  // m
            INTERWHEEL_DISTANCE  = 0.053f;  // m
            WHEEL_RADIUS = 0.0205f;  // m

            m_cNoTurnOnAngleThreshold   = ToRadians(CDegrees(10.0f)); //10.0 - straight to food spot; 35.0 spiral to food spot
            m_cSoftTurnOnAngleThreshold = ToRadians(CDegrees(70.0f));
        }

        void SetKinematicDetails(Real f_MaxLeftWheelSpeed, Real f_MaxRightWheelSpeed) // arguments are speeds in cm/s
        {
            MaxLinearSpeed        = ((f_MaxLeftWheelSpeed + f_MaxRightWheelSpeed) / 2.0f) * seconds_per_iterations;  //in cm/ (controlcycle)

            // as the max speed is 3 cm/controlcycle (resultant speed is always positive as the robot does not traverse backwards), the max acceleration is +/-3 cm/controlcycle/controlcycle
            // so, MaxLinearAcceleration = |+/-3 cm/controlcycle/controlcycle| = 3 cm/controlcycle/controlcycle
            MaxLinearAcceleration = MaxLinearSpeed;

            // the max angular speed is +/- X degrees/controlcycle,
            // so MaxAngularSpeed = |+/- X degrees/controlcycle| = |X degrees/controlcycle|
            MaxAngularSpeed       = ((f_MaxLeftWheelSpeed + f_MaxRightWheelSpeed) /
                                     (INTERWHEEL_DISTANCE * 100.0f)) * seconds_per_iterations; //rad/controlcycle

            // as the max angular speed is +/-X degrees/controlcycle, the max acceleration is +/- 2X radians/controlcycle/controlcycle
            // so MaxAngularAcceleration = |+/- 2X degrees/controlcycle/controlcycle| = 2X degrees/controlcycle/controlcycle
            MaxAngularAcceleration   = 2.0f * MaxAngularSpeed; //rad/controlcycle/controlcycle;
        }
    };

    RobotDetails m_sRobotDetails;

public:

    CEPuckHomSwarm();
    virtual ~CEPuckHomSwarm();

    virtual void CopyRobotDetails(RobotDetails& sRobotDetails);

    CCI_EPuckBaseLEDsActuator* GetLEDsPtr() {return m_pcLEDs;}

    /**
     * Run one of the swarm experiments (Aggregation, Dispersion, Homing, Flocking)
    */
    virtual void RunHomogeneousSwarmExperiment();

    /**
     *  This function returns the interger cast of the string robot id
    */
    virtual unsigned RobotIdStrToInt();

    /**
    * Returns the experiment type
    */
    inline ExperimentToRun& GetExperimentType()
    {
        return m_sExpRun;
    }


    virtual void Init(TConfigurationNode& t_node);
    virtual void ControlStep();
    virtual void Reset();
    virtual void Destroy() {}

    Real m_fInternalRobotTimer; Real m_fRobotTimerAtStart;

    std::vector <int> beaconrobots_ids;

private:

    CCI_EPuckProximitySensor::TReadings GetIRSensorReadings()
    {
        //CCI_EPuckProximitySensor::TReadings sensor_readings = m_pcProximity->GetReadings();
		CCI_EPuckProximitySensor::TReadings sensor_readings;

        for (size_t i = 0; i < sensor_readings.size(); ++i)
            sensor_readings[i].Value /= 4096.0f; // normalize sensor reading to range <0,1>

        return sensor_readings;
    }


    CCI_EPuckPseudoRangeAndBearingSensor::TPackets GetRABSensorReadings()
    {
        CCI_EPuckPseudoRangeAndBearingSensor::TPackets sensor_readings = m_pcRABS->GetPackets();

        return sensor_readings;
    }
	
	CCI_EPuckLightSensor::TReadings GetLightSensorReadings() 
	{
		CCI_EPuckLightSensor::TReadings sensor_readings = m_pcLight->GetReadings();
		
		return sensor_readings;
	}

private:

    // Sensors
    CCI_EPuckProximitySensor* m_pcProximity;
    CCI_EPuckPseudoRangeAndBearingSensor* m_pcRABS;
	CCI_EPuckLightSensor* m_pcLight;

    // Actuators
    CCI_EPuckWheelsActuator* m_pcWheels;
    CCI_EPuckBaseLEDsActuator* m_pcLEDs;

    TBehaviorVector             m_vecBehaviors;
    bool                        b_damagedrobot;     // true if robot is damaged

    CFlockingBehavior*          m_pFlockingBehavior;

    unsigned                    u_num_consequtivecollisions;

    /* The random number generator */
    CRandom::CRNG* m_pcRNG;
    CRandom::CRNG* m_pcRNG_FVs; // we have a separate RNG for forgetting FVs. This way the actual behaviors based on random walk are independent from the running of the CRM.

    /*Information on the experiment to run - the swarm behavior and the faulty behavior*/
    ExperimentToRun m_sExpRun;

    unsigned m_uRobotId, m_uRobotFV; // Robot Id and proprioceptively computed FV

    Real leftSpeed, rightSpeed, leftSpeed_prev, rightSpeed_prev;

    bool b_randompositionrobot; // seeding the expeiment with random initial positions for the robots

    bool m_bRobotSwitchedBehavior; // used to indicate if the robot has switched behavior for swarm behavior transition experiments
};

#endif

