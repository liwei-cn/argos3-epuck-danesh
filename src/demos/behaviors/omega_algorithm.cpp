#include "omega_algorithm.h"

static const Real WHEEL_RADIUS = 0.0205f;
static const Real WHEEL_CIRCUMFERENCE = 2 * CRadians::PI.GetValue() * WHEEL_RADIUS;  //0.1287
static const Real INTERWHEEL_DISTANCE = 0.053f;

static const Real INTERWHEEL_CIRCUMFERENCE = INTERWHEEL_DISTANCE * CRadians::PI.GetValue();
static const Real DISTANCE_PER_DEGREE = INTERWHEEL_CIRCUMFERENCE / 360 * 100; //In centimetres

static const Real MAX_SPEED = WHEEL_CIRCUMFERENCE * 100; //12.88 cm/s

using namespace std;

CEPuckOmegaAlgorithm::CEPuckOmegaAlgorithm(Real m_fRangeAndBearing_RangeThreshold):
		m_fRangeAndBearing_RangeThreshold(m_fRangeAndBearing_RangeThreshold),
        state(forward),
        aggregation_timer(0),
        distance_turned(0),
		omega(25),
		shadowed_avoidance_radius(0.1),
		seconds_per_tick(0.1),
		illuminated_avoidance_radius(0.2),
		avoidance_radius(0),
		illuminated(false)
		{
		};

bool CEPuckOmegaAlgorithm::TakeControl(Real m_fInternalRobotTimer)
{

	printf("\n\nstate: %d control step: %f\n", state, m_fInternalRobotTimer);
    // Determine illumination status

    illuminated = false;
    int count = 0;

	printf("[LIGHT]\t\t");
    for(size_t i = 0; i <  m_sSensoryData.m_LightSensorData.size(); ++i)
    {
        printf("%.2f, ", m_sSensoryData.m_LightSensorData[i].Value);
	}
	printf("\n");
	
    for(size_t i = 0; i <  m_sSensoryData.m_LightSensorData.size(); ++i)
    {
		if(m_sSensoryData.m_LightSensorData[i].Value > 0)
        {
            count++;

            // Robot is only 'illuminated' if 3 or more of the light sensors can see the beacon
            if(count >= 3)
            {
                illuminated = true;
                break;
            }
        }
    }

    // Set the robot's avoidance radius according to its illumination status
    // The LEDs also change based on illumination status, but only for visual debugging purposes
    if(illuminated)
    {
    //    leds->SetAllColors(CColor::YELLOW);
        avoidance_radius = illuminated_avoidance_radius;
    }
    else
    {
    //    leds->SetAllColors(CColor::BLACK);
        avoidance_radius = shadowed_avoidance_radius;
    }

    printf("avoidance radius: %f\n", avoidance_radius);
	
    // Omega algorithm state machine
    if(state == forward)
    {
        // Bjerknes programmed the e-pucks to move at a quarter of their maximum speed
        left_motor_speed = right_motor_speed = MAX_SPEED / 4;

        // Perform coherence if the aggregation timer has expired
        if(m_fInternalRobotTimer - aggregation_timer > omega)
        {
			printf("RAB size: %d\n", m_sSensoryData.m_RABSensorData.size());
        	if(m_sSensoryData.m_RABSensorData.size() == 0)
                heading.SetValue(180);  //the robot takes 2.6s (26 time steps) to turn 180 degrees
            else
            {
                CVector2 centroid;

                for(int i = 0; i < m_sSensoryData.m_RABSensorData.size(); ++i)
                {
					printf("[RAB]\t\t");
                    //const CCI_EPuckRangeAndBearingSensor::SPacket packet = m_sSensoryData.m_RABSensorData[i];
					if(m_sSensoryData.m_RABSensorData[i]->Range < m_fRangeAndBearing_RangeThreshold) 
					{
						printf("RobotId:%d, Range:%.2f, Bearing:%.2f\t", m_sSensoryData.m_RABSensorData[i]->RobotId, m_sSensoryData.m_RABSensorData[i]->Range, ToDegrees(m_sSensoryData.m_RABSensorData[i]->Bearing).GetValue());
						printf("\n\t\t");
						
						Real range = m_sSensoryData.m_RABSensorData[i]->Range;
                    	CRadians bearing = m_sSensoryData.m_RABSensorData[i]->Bearing;

                    	// No need to ever divide by the number of robots - we only care about the angle, not the actual coordinates
                    	centroid += CVector2(range, bearing);
					}
                }
				printf("\n");
                // Calculate the angle towards the swarm centroid
                heading = ToDegrees(centroid.Angle());
            }

            state = coherence;
        }
        else if(m_fInternalRobotTimer - aggregation_timer > 5) // Otherwise, check whether avoidance is required (with some cool-off period since the last turn)
        {
            CVector2 centroid;

            for(int i = 0; i < m_sSensoryData.m_RABSensorData.size(); ++i)
            {
				printf("[RAB] \t\t");
                //const CCI_RangeAndBearingSensor::SPacket packet = m_sSensoryData.m_RABSensorData[i];
				if(m_sSensoryData.m_RABSensorData[i]->Range < m_fRangeAndBearing_RangeThreshold) 
				{
					
					printf("RobotId:%d, Range:%.2f, Bearing:%.2f\t", m_sSensoryData.m_RABSensorData[i]->RobotId, m_sSensoryData.m_RABSensorData[i]->Range, ToDegrees(m_sSensoryData.m_RABSensorData[i]->Bearing).GetValue());
					printf("\n\t\t");
					
					Real range = m_sSensoryData.m_RABSensorData[i]->Range / 100; // Convert centimetres to metres

					// Only avoid robots within our avoidance radius
					if(range < avoidance_radius)
					{
						CRadians bearing = m_sSensoryData.m_RABSensorData[i]->Bearing;

						centroid += CVector2(range, bearing);
					}
				}
				printf("\n");
            }

            // Only avoid if there were robots close enough
            if(centroid.Length() != 0)
            {
                // Turn 180 degrees away from the robots being avoided
                centroid.Rotate(ToRadians(CDegrees(180)));

                heading = ToDegrees(centroid.Angle());

                state = avoidance;
            }
        }
    }
    else if(state == avoidance || state == coherence)
    {
    	//printf("state: %d control step: %f\n", state, m_fInternalRobotTimer);
    	// Either turn left or right, depending on the desired heading
        if(heading.GetValue() <= 0)
        {
            left_motor_speed = MAX_SPEED / 4;
            right_motor_speed = -MAX_SPEED / 4;
        }
        else
        {
            left_motor_speed = -MAX_SPEED / 4;
            right_motor_speed = MAX_SPEED / 4;
        }

        // Use the previous desired left motor speed to calculate how far the robot has turned
        distance_turned += std::abs(left_motor_speed * seconds_per_tick);

        // Stop turning once the desired heading has been reached
        if(distance_turned >= std::abs(DISTANCE_PER_DEGREE * heading.GetValue()))
        {
            distance_turned = 0;
            aggregation_timer = m_fInternalRobotTimer;
            state = forward;
        }
    }

	return true;

}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CEPuckOmegaAlgorithm::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
	fLeftWheelSpeed = left_motor_speed;
	fRightWheelSpeed = right_motor_speed;
}

/******************************************************************************/
/******************************************************************************/

void CEPuckOmegaAlgorithm::PrintBehaviorIdentity()
{
    std::cout << "Omega Algorithm taking over" << std::endl;
}


