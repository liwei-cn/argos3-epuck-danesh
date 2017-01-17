#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

CBehavior::CBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

CBehavior::~CBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

void CBehavior::Suppress()
{}

/******************************************************************************/
/******************************************************************************/

void CBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{}

/******************************************************************************/
/******************************************************************************/

void CBehavior::PrintBehaviorIdentity()
{}

/******************************************************************************/
/******************************************************************************/

void CBehavior::WheelSpeedsFromHeadingVector(CVector2 &m_cHeadingVector, Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    /* Get the heading angle */
    CRadians cHeadingAngle = m_cHeadingVector.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = m_cHeadingVector.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sRobotData.MaxSpeed);

    int TurningMechanism;

    /* Turning state switching conditions */
    if(cHeadingAngle.GetAbsoluteValue() <= m_sRobotData.m_cNoTurnOnAngleThreshold.GetValue())
    {
        /* No Turn, heading angle very small */
        TurningMechanism = NO_TURN;
        //std::cout << " cHeadingAngle " << cHeadingAngle << " fBaseAngularWheelSpeed " << fBaseAngularWheelSpeed << " NO_TURN " << std::endl;
    }
    else if(cHeadingAngle.GetAbsoluteValue() > m_sRobotData.m_cNoTurnOnAngleThreshold.GetValue() &&
            cHeadingAngle.GetAbsoluteValue() <= m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue())
    {
        /* Soft Turn, heading angle in between the two cases */
        TurningMechanism = SOFT_TURN;
        //std::cout << " cHeadingAngle " << cHeadingAngle << " fBaseAngularWheelSpeed " << fBaseAngularWheelSpeed << " SOFT_TURN " << std::endl;
    }
    else if(cHeadingAngle.GetAbsoluteValue() > m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue()) // m_sWheelTurningParams.SoftTurnOnAngleThreshold
    {
        /* Hard Turn, heading angle very large */
        TurningMechanism = HARD_TURN;
        //std::cout << " cHeadingAngle " << cHeadingAngle << " fBaseAngularWheelSpeed " << fBaseAngularWheelSpeed << " HARD_TURN " << std::endl;
    }


    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(TurningMechanism)
    {
    case NO_TURN:
    {
        /* Just go straight */
        fSpeed1 = fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
        break;
    }

    case SOFT_TURN:
    {
        /* Both wheels go straight, but one is faster than the other */ //HardTurnOnAngleThreshold
        Real fSpeedFactor = (m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue() - cHeadingAngle.GetAbsoluteValue()) /
                m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue();
        fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
        fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);


        /*
         * to make sure the speeds do not exceed the max speed
         *
         */
        if (fSpeed1 > m_sRobotData.MaxSpeed)
            fSpeed1 = m_sRobotData.MaxSpeed;

        if (fSpeed2 > m_sRobotData.MaxSpeed)
            fSpeed2 = m_sRobotData.MaxSpeed;

        if (fSpeed1 < -m_sRobotData.MaxSpeed)
            fSpeed1 = -m_sRobotData.MaxSpeed;

        if (fSpeed2 < -m_sRobotData.MaxSpeed)
            fSpeed2 = -m_sRobotData.MaxSpeed;


        break;
    }

    case HARD_TURN:
    {
        /* Opposite wheel speeds */
        fSpeed1 = -m_sRobotData.MaxSpeed;
        fSpeed2 =  m_sRobotData.MaxSpeed;
        break;
    }
    }

    /* Apply the calculated speeds to the appropriate wheels */
    if(cHeadingAngle > CRadians::ZERO)
    {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else
    {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
}

/******************************************************************************/
/******************************************************************************/
