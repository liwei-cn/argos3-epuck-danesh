#ifndef EPUCK_OMEGA_ALGORITHM_H
#define EPUCK_OMEGA_ALGORITHM_H

#include "behavior.h"
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CEPuckOmegaAlgorithm : public CBehavior
{

public:
    CEPuckOmegaAlgorithm(Real m_fRangeAndBearing_RangeThreshold);

    void SimulationStep()
    {

    }


	virtual bool TakeControl() {};
    virtual bool TakeControl(Real m_fInternalRobotTimer);
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

protected:
    Real           m_fRangeAndBearing_RangeThreshold;
	
private:
	enum State {forward, avoidance, coherence};
	State state;

	double omega;
    double avoidance_radius;
    double shadowed_avoidance_radius;
	double illuminated_avoidance_radius;
	int aggregation_timer;
	double distance_turned;
	CDegrees heading;
	
	double left_motor_speed;
	double right_motor_speed;

    double seconds_per_tick;

};

#endif
