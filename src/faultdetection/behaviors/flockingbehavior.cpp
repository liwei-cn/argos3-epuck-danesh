#include "flockingbehavior.h"
#include "assert.h"
#include <iostream>


/******************************************************************************/
/******************************************************************************/
#define ROBOTID_NOT_IN_SIGNAL          999
#define ROBOTSELFBEARING_NOT_IN_SIGNAL 999.0f
#define ROBOTSELFACCELERATION_NOT_IN_SIGNAL 999.0f
/******************************************************************************/
/******************************************************************************/

CFlockingBehavior::CFlockingBehavior(unsigned VelocityTimeWindowLength) :
    m_cFlockingVector(0.0, 0.0),
    m_uVelocityTimeWindowLength(VelocityTimeWindowLength)
{
}

/******************************************************************************/
/******************************************************************************/

bool CFlockingBehavior::TakeControl_RABEstimatedOdometry()
{
    bool controltaken(false);

    m_cFlockingVector.Set(0.0f, 0.0f);
    unsigned robotsinrange_flock = 0;


    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    for (it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
    {
        // Only update the FlockingVector using robots you are observing
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            unsigned u_ObservedRobotId = m_sSensoryData.m_RABSensorData[i]->RobotId;
            if(u_ObservedRobotId == (it_listobrob->m_unRobotId))
            {
                if (it_listobrob->GetVelocity_MediumTimeWindow().GetX() != -10000.0f)// && m_sSensoryData.m_RABSensorData[i].Range < 25.0f)
                {
                    m_cFlockingVector +=it_listobrob->GetVelocity_MediumTimeWindow();
                    robotsinrange_flock++;
                }
            }
        }
    }


    m_fAvgProximityNbrs = 0.0f;
    m_cAggregationVector.Set(0.0f, 0.0f);
    unsigned robotsinrange_aggr = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        m_cAggregationVector += CVector2(m_sSensoryData.m_RABSensorData[i]->Range, m_sSensoryData.m_RABSensorData[i]->Bearing);
        m_fAvgProximityNbrs  += m_sSensoryData.m_RABSensorData[i]->Range;
        robotsinrange_aggr++;
    }

    //if(robotsinrange_aggr > 3)
    if(robotsinrange_aggr > 1)
    {
        m_cAggregationVector /= robotsinrange_aggr;
        m_fAvgProximityNbrs = m_fAvgProximityNbrs / robotsinrange_aggr;
    }
    else
    {
        m_cAggregationVector = CVector2(0.0f, 0.0f);
        m_fAvgProximityNbrs = 0.0f;
    }


    if(robotsinrange_flock > 0u)
    {
        m_cFlockingVector /= robotsinrange_flock;
        return true;
    }
    else
        return false;
}

/******************************************************************************/
/******************************************************************************/    

bool CFlockingBehavior::TakeControl()
{
    bool controltaken(false);

    m_cFlockingVector.Set(0.0f, 0.0f);
    unsigned robotsinrange_flock = 0;
	
   // Only update the FlockingVector using robots you are observing
	for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
	{
		if(m_sSensoryData.m_RABSensorData[i]->Data[1] == 1)
		{
			m_cFlockingVector += CVector2((Real)(m_sSensoryData.m_RABSensorData[i]->Data[2])/10.0f, 
										  (Real)(m_sSensoryData.m_RABSensorData[i]->Data[3])/10.0f);
			robotsinrange_flock++;
		}
	}

    m_fAvgProximityNbrs = 0.0f;
    m_cAggregationVector.Set(0.0f, 0.0f);
    unsigned robotsinrange_aggr = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        m_cAggregationVector += CVector2(m_sSensoryData.m_RABSensorData[i]->Range, m_sSensoryData.m_RABSensorData[i]->Bearing);
        m_fAvgProximityNbrs  += m_sSensoryData.m_RABSensorData[i]->Range;
        robotsinrange_aggr++;
    }

    //if(robotsinrange_aggr > 3)
    if(robotsinrange_aggr > 1)
    {
        m_cAggregationVector /= robotsinrange_aggr;
        m_fAvgProximityNbrs = m_fAvgProximityNbrs / robotsinrange_aggr;
    }
    else
    {
        m_cAggregationVector = CVector2(0.0f, 0.0f);
        m_fAvgProximityNbrs = 0.0f;
    }


    if(robotsinrange_flock > 0u)
    {
        m_cFlockingVector /= robotsinrange_flock;
        return true;
    }
    else
        return false;
}

/******************************************************************************/
/******************************************************************************/

// Move at average velocity of your neighbours
void CFlockingBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    //if neighbours very far away, put more weight on aggregation; else put more weight on flocking

    m_fAvgProximityNbrs /= 100.0f; // normalise - max range is 100cm.
	
	assert(m_fAvgProximityNbrs <= 1.0f);

    // m_fAvgProximityNbrs = m_fAvgProximityNbrs*m_fAvgProximityNbrs; // enable to have flocks moving faster, but with more distance between robots of aggregate

     //CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed * (0.75f * m_cFlockingVector.Normalize() + 0.25f * m_cAggregationVector.Normalize());
     CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed * ((1.0f - m_fAvgProximityNbrs) * m_cFlockingVector.Normalize() + m_fAvgProximityNbrs * m_cAggregationVector.Normalize());

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);

     //fLeftWheelSpeed = (fLeftWheelSpeed + m_sSensoryData.f_LeftWheelSpeed)/2.0f;
     //fRightWheelSpeed = (fRightWheelSpeed + m_sSensoryData.f_RightWheelSpeed)/2.0f;
}

/******************************************************************************/
/******************************************************************************/

void CFlockingBehavior::PrintBehaviorIdentity()
{
    std::cout << "Flocking taking over";
}

/******************************************************************************/
/******************************************************************************/

CFlockingBehavior::ObservedNeighbours::ObservedNeighbours(CFlockingBehavior &owner_class, Real TimeFirstObserved, unsigned ObservedRobotId):
    owner(owner_class), m_fTimeFirstObserved(TimeFirstObserved), m_unRobotId(ObservedRobotId)
{
    /************************************************************************************/
    m_fEstimated_Velocity_ShortTimeWindow = CVector2(-10000.0f, -10000.0f); m_fEstimated_Velocity_MediumTimeWindow = CVector2(-10000.0f, -10000.0f);
    m_fEstimated_Velocity_LongTimeWindow = CVector2(-10000.0f, -10000.0f);

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    /************************************************************************************/

     u_TimeSinceLastObserved = 0u; u_TimeSinceLastObserved_DistMeasure = 0u;
}

/******************************************************************************/
/******************************************************************************/

/* Use the copy constructor since your structure has raw pointers and push_back the structure instance to list will just copy the pointer value */
CFlockingBehavior::ObservedNeighbours::ObservedNeighbours(const ObservedNeighbours &ClassToCopy):
    owner(ClassToCopy.owner), m_fTimeFirstObserved(ClassToCopy.m_fTimeFirstObserved), m_unRobotId(ClassToCopy.m_unRobotId)
{
    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    m_fEstimated_Velocity_ShortTimeWindow = CVector2(-10000.0f, -10000.0f); m_fEstimated_Velocity_MediumTimeWindow = CVector2(-10000.0f, -10000.0f);
    m_fEstimated_Velocity_LongTimeWindow = CVector2(-10000.0f, -10000.0f);

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    /************************************************************************************/


    u_TimeSinceLastObserved = 0u; u_TimeSinceLastObserved_DistMeasure = 0u;
}

/******************************************************************************/
/******************************************************************************/

CFlockingBehavior::ObservedNeighbours::~ObservedNeighbours()
{
    vec_RobPos_ShortRangeTimeWindow.clear();
    vec_RobPos_MediumRangeTimeWindow.clear();
    vec_RobPos_LongRangeTimeWindow.clear();
}

/******************************************************************************/
/******************************************************************************/

void CFlockingBehavior::SimulationStep()
{
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        /* this loop is quadratic in computational complexity. we can improve on this.*/

        bool     b_ObservedRobotFound(false);
        unsigned u_ObservedRobotId = m_sSensoryData.m_RABSensorData[i]->RobotId;
        for (t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
        {
            if(u_ObservedRobotId == it_listobrob->m_unRobotId)
            {
                b_ObservedRobotFound = true;
                break; // each robot id is represented only once in the m_pcListObservedRobots list
            }
        }

        /* add the robot u_ObservedRobotId to the list of observed robots*/
        if (!b_ObservedRobotFound)
            m_pcListObservedRobots.push_back(ObservedNeighbours((*this), m_sSensoryData.m_rTime, u_ObservedRobotId));
    }


    /*
    But when do we delete the observed robot from the list of observed robots, if it has not been observed for a long time.
    For now we don't delete it.
    */
    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    while (it_listobrob != m_pcListObservedRobots.end())
    {
        /*
         * Gets observation of distance travelled by robot in last 1s, 5s, and 10s
           Robots that are not observed in the current time-step are merely marked as such instead of clearing the entire window if even one observation is missed
        */
        it_listobrob->EstimateOdometry();

        ++it_listobrob;
    }
}

/******************************************************************************/
/******************************************************************************/

void CFlockingBehavior::ObservedNeighbours::EstimateOdometry()
{
    Real observedRobotId_1_Range; CRadians observedRobotId_1_Bearing;

    bool b_DataAvailable = GetObservedRobotRangeBearing(observedRobotId_1_Range, observedRobotId_1_Bearing);
    // assert(b_DataAvailable); // WHAT IF THE ROBOT IS NOT OBSERVED AT THE CURRENT TIME-STEP

    /*
     * Computing angle rotated by robot in one tick
    */
    CRadians delta_orientation = CRadians(owner.m_sRobotData.seconds_per_iterations * ((-owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed)
                                                                                       / (owner.m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));

    /*if (owner.m_sSensoryData.m_unRobotId == 0u)
        printf("%f\t",ToDegrees(delta_orientation).GetValue());*/


    Real step = owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved;


    assert(step >= 0.0f);


    m_fEstimated_Velocity_ShortTimeWindow  = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_ShortRangeTimeWindow, b_DataAvailable);
    m_fEstimated_Velocity_MediumTimeWindow = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_MediumRangeTimeWindow, b_DataAvailable);
    m_fEstimated_Velocity_LongTimeWindow   = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_LongRangeTimeWindow, b_DataAvailable);
}

/******************************************************************************/
/******************************************************************************/

CVector2 CFlockingBehavior::ObservedNeighbours::TrackRobotDisplacement(Real step, Real observedRobotId_1_Range, CRadians observedRobotId_1_Bearing, CRadians delta_orientation, std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable)
{
    CVector2 displacement(-10000.0f, -10000.0f); /* if step is < displacement_vector.size, we return -10000 as not enough time has elapsed to make an observation */

    assert(step >= 0.0f);

    /*if(m_unRobotId == 16 && owner.m_sSensoryData.m_unRobotId ==19 && displacement_vector.size() == 100)
    {
        printf("step %f, b_DataAvailable %d, observedRobotId_1_Range %f, observedRobotId_1_Bearing %f, delta_orientation %f, l_speed %f r_speed %f\n\n",step, b_DataAvailable,observedRobotId_1_Range, observedRobotId_1_Bearing.GetValue(), delta_orientation.GetValue(), owner.m_sSensoryData.f_LeftWheelSpeed, owner.m_sSensoryData.f_RightWheelSpeed);
    }*/

    /*
     * Returns magnitude of displacement vector of the robot in the pre-specified time interval. If the robot is unobservable at the end of the pre-specified time interval, the function returns -1.
     */
    if(step < (Real)displacement_vector.size())
    {

        for (size_t t = 0 ; t < (unsigned)(step); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }

        displacement_vector[(unsigned)(step)].b_DataAvailable = b_DataAvailable;
        if (displacement_vector[(unsigned)(step)].b_DataAvailable)
        {
            displacement_vector[(unsigned)(step)].Range_At_Start           = observedRobotId_1_Range;
            displacement_vector[(unsigned)(step)].Bearing_At_Start         = observedRobotId_1_Bearing;
            displacement_vector[(unsigned)(step)].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing),
                                                                   observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        }
        else
        {
            displacement_vector[(unsigned)(step)].Range_At_Start           = -1.0f;
            displacement_vector[(unsigned)(step)].Bearing_At_Start.SetValue(-1.0f);
            displacement_vector[(unsigned)(step)].Pos_At_Start.Set(-1.0f, -1.0f);
        }


        displacement_vector[(unsigned)(step)].TimeSinceStart           = 0.0f;
        displacement_vector[(unsigned)(step)].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[(unsigned)(step)].NetRotationSinceStart.SetValue(0.0f);
    }
    else
    {
        for (size_t t = 0 ; t < displacement_vector.size(); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }


        size_t t = ((unsigned)(step)%displacement_vector.size());
        assert(t < displacement_vector.size());



        if(displacement_vector[(unsigned)(t)].b_DataAvailable && b_DataAvailable)
        {
            CVector2 tmp_pos           = CVector2(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
            CVector2 tmp_pos_rot       = tmp_pos.Rotate(displacement_vector[t].NetRotationSinceStart);
            CVector2 pos_rot_trans     = tmp_pos_rot + displacement_vector[t].NetTranslationSinceStart;

            CVector2 Pos_At_Start      = displacement_vector[t].Pos_At_Start;

            /*
             * Computing average displacement
             */
            displacement              = (pos_rot_trans - Pos_At_Start);
        }
        else
        {
            /*
                We don't have the range and bearing observations of the robot to position it at the end of the pre-specified time interval. So we can't compute the displacement.
            */
            displacement              = CVector2(-10000.0f, -10000.0f);
        }

        /*if(m_unRobotId == 16 && owner.m_sSensoryData.m_unRobotId ==19 && displacement_vector.size() == 100)
        {
            printf("displacement %f \n", displacement);
        }*/


        /* Preparing the start recorded data to be used at the end of the pre-specified time interval */
        displacement_vector[(unsigned)(t)].b_DataAvailable = b_DataAvailable;
        if (displacement_vector[(unsigned)(t)].b_DataAvailable)
        {
            displacement_vector[t].Range_At_Start           = observedRobotId_1_Range;
            displacement_vector[t].Bearing_At_Start         = observedRobotId_1_Bearing;
            displacement_vector[t].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        }
        else
        {
            displacement_vector[t].Range_At_Start           = -1.0f;
            displacement_vector[t].Bearing_At_Start.SetValue(-1.0f);
            displacement_vector[t].Pos_At_Start.Set(-1.0f, -1.0f);
        }
        displacement_vector[t].TimeSinceStart = 0.0f;
        displacement_vector[t].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[t].NetRotationSinceStart.SetValue(0.0f);
    }


    //std::cout << "Displacement for robot id " << m_unRobotId << " is " << displacement << std::endl;
    return displacement;
}

/******************************************************************************/
/******************************************************************************/

bool CFlockingBehavior::ObservedNeighbours::GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing)
{
    bool observedRobotFound(false);
    for(size_t i = 0; i <  owner.m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_unRobotId == owner.m_sSensoryData.m_RABSensorData[i]->RobotId)
        {
            observedRobotId_1_Range   = owner.m_sSensoryData.m_RABSensorData[i]->Range;
            observedRobotId_1_Bearing = owner.m_sSensoryData.m_RABSensorData[i]->Bearing;
            observedRobotFound = true;

            break;
        }
    }

    return observedRobotFound;
}

/******************************************************************************/
/******************************************************************************/
