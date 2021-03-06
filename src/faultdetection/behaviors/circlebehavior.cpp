#include "circlebehavior.h"
//#include "random.h"


/******************************************************************************/
/******************************************************************************/

CCircleBehavior::CCircleBehavior()
{    
}

/******************************************************************************/
/******************************************************************************/
    
bool CCircleBehavior::TakeControl()
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CCircleBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    // Circle with radius INTERWHEEL_DISTANCE - 5.3 cm
    fLeftWheelSpeed  = 0.0f;
    fRightWheelSpeed = m_sRobotData.MaxSpeed;
}

/******************************************************************************/
/******************************************************************************/

void CCircleBehavior::PrintBehaviorIdentity()
{
    std::cout << "Circle behavior taking over";
}

/******************************************************************************/
/******************************************************************************/
