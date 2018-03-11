#ifndef SENSINGANDCOMMUNICATION_H_
#define SENSINGANDCOMMUNICATION_H_

//#define DEBUG_FV_MESSAGES

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>    // std::sort

/****************************************/
/****************************************/
/* ARGoS headers */

/* Deubg message flags*/
#include <argos3/plugins/robots/e-puck/real_robot/real_epuck_debugmessages.h>

/* Definition of the pseudo range and bearing sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_pseudo_range_and_bearing_sensor.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/****************************************/
/****************************************/

/* Definition of functions to estimate feature-vectors - proprioceptively, or by observation */

#include "propriofeaturevector.h"

#include "bayesianinferencefeaturevector.h"

/****************************************/
/****************************************/
/* Definition of functions to assimilate the different feature-vectors and perform abnormality detection */

#include "featurevectorsinrobotagent.h"

/****************************************/
/****************************************/

#define FV_MODE BAYESIANINFERENCE_MODE

/****************************************/
/****************************************/

/*
 * Probability to forget FV in distribution
 */
#define PROBABILITY_FORGET_FV 1.0f //0.001f // We don't need a large history because FVs are being relayed every time-step. That combined with the BI of FVs (small observation window) means that robots will have sufficient FVs of neighbours to make a decision. Also it is difficult to assume that robot behavior has not changed in the last 100s.Will have a CRM_RESULTS_VALIDFOR_SECONDS history instead.

/*
 * The results of the CRM are valid for atmost 10s in the absence of any FVs to run the CRM
 */
#define CRM_RESULTS_VALIDFOR_SECONDS 10.0f // in seconds // assume the robot behaviors have not changed in the last 10s // can we remove this.

/****************************************/
/****************************************/


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;


/*
 *
 */
void Sense(Real m_fProbForget,
           unsigned RobotId, Real m_fInternalRobotTimer, CCI_EPuckPseudoRangeAndBearingSensor::TPackets &rabsensor_readings,
           t_listMapFVsToRobotIds& listMapFVsToRobotIds, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listFVsSensed& listFVsSensed,
           CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
           CRandom::CRNG* m_pcRNG_FVs, std::string &swarmbehav, std::vector<int> &beaconrobots_ids);

/*
 *
 */
void SenseCommunicateDetect(std::ofstream &m_cOutput, unsigned RobotId, Real leftSpeed_prev, Real rightSpeed_prev,
                            Real m_fInternalRobotTimer, CCI_EPuckPseudoRangeAndBearingSensor::TPackets &rabsensor_readings,
                            t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listMapFVsToRobotIds& MapFVsToRobotIds_Relay, t_listFVsSensed& CRMResultsOnFVDist,
                            CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                            CRandom::CRNG* m_pcRNG_FVs, unsigned& m_uRobotFV, std::string &swarmbehav, std::vector<int> &beaconrobots_ids);


#endif
