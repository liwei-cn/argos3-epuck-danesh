#include "dispersebehavior.h"


/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior(Real m_fProximitySensorThreshold, CRadians m_cGoStraightAngleThreshold) :
    m_fProximitySensorThreshold(m_fProximitySensorThreshold),
    m_cGoStraightAngleThreshold(m_cGoStraightAngleThreshold)
{
}

/******************************************************************************/
/******************************************************************************/

bool CDisperseBehavior::TakeControl() 
{
    /* Get readings from proximity sensor */
    /* Sum them together */
    m_cDiffusionVector.Set(0.0f, 0.0f);
    for(size_t i = 0; i <  m_sSensoryData.m_ProximitySensorData.size(); ++i)
    {
        m_cDiffusionVector += CVector2(m_sSensoryData.m_ProximitySensorData[i].Value, m_sSensoryData.m_ProximitySensorData[i].Angle);
    }
    m_cDiffusionVector /= m_sSensoryData.m_ProximitySensorData.size();


    /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
    if(m_cDiffusionVector.Angle().GetAbsoluteValue() < m_cGoStraightAngleThreshold.GetValue() && m_cDiffusionVector.Length() < m_fProximitySensorThreshold)
    {
        return false;
    }
    else
    {
        if(m_cDiffusionVector.Length() < 0.05) /* because of noise, we can have very small non-zero sensor readings. but we don't want to responmd to them*/
            return false;

        //std::cout << " m_cDiffusionVector length " << m_cDiffusionVector.Length() << " and threshold " << m_fProximitySensorThreshold << std::endl;
        //std::cout << " m_cDiffusionVector angle " <<  m_cDiffusionVector.Angle().GetAbsoluteValue() << " and threshold " << m_cGoStraightAngleThreshold.GetValue() << std::endl;

        return true;
    }
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CDisperseBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    CVector2 m_cHeadingVector = -m_cDiffusionVector.Normalize() * m_sRobotData.MaxSpeed;

    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::PrintBehaviorIdentity()
{
    std::cout << "Disperse taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

