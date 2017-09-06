
#include "sensingandcommunication.h"



/****************************************/
/****************************************/

//Sense(PROBABILITY_FORGET_FV, RobotId, m_fInternalRobotTimer, rabsensor_readings,
//      listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed,
//      m_cBayesianInferredFeatureVector, m_pcRNG_FVs, swarmbehav, beaconrobots_ids);


void Sense(Real m_fProbForget, unsigned RobotId, Real m_fInternalRobotTimer,
           CCI_EPuckPseudoRangeAndBearingSensor::TPackets &rabsensor_readings,
           t_listMapFVsToRobotIds& listMapFVsToRobotIds, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listFVsSensed& listFVsSensed,
           CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
           CRandom::CRNG* m_pcRNG_FVs, std::string &swarmbehav, std::vector<int> &beaconrobots_ids)
{
#if FV_MODE == BAYESIANINFERENCE_MODE
    for (size_t i = 0; i < m_cBayesianInferredFeatureVector.ObservedRobotIDs.size(); ++i)
    {
        /* Ignore the homing beacon in fault detection */
        if(swarmbehav.compare("SWARM_HOMING") == 0)
            if(m_cBayesianInferredFeatureVector.ObservedRobotIDs[i] == 0u)
            {
                continue;
            }

        /* Ignore the foraging beacon(s) in fault detection */
        if(swarmbehav.compare("SWARM_FORAGING") == 0)
            if(std::find(beaconrobots_ids.begin(), beaconrobots_ids.end(), m_cBayesianInferredFeatureVector.ObservedRobotIDs[i]) != beaconrobots_ids.end())
            {
                continue;
            }



        unsigned robotId = m_cBayesianInferredFeatureVector.ObservedRobotIDs[i];
        unsigned fv      = m_cBayesianInferredFeatureVector.ObservedRobotFVs[i];


        unsigned num_obs_sm  = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Number_Featureobservations_SM[i];
        unsigned num_obs_nsm = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Number_Featureobservations_nSM[i];
        unsigned num_obs_m   = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Number_Featureobservations_M[i];




        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer,
                             m_cBayesianInferredFeatureVector.ObservedRobotIDs_range[i],
                             num_obs_sm, num_obs_nsm, num_obs_m);
    }

    //remove entries older than 10s
    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBayesianInferenceFeatureVector::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);


    /*if((RobotIdStrToInt() == 6 || RobotIdStrToInt() == 13) && ((m_fInternalRobotTimer == 1701.0f || m_fInternalRobotTimer == 1702.0f)  || (m_fInternalRobotTimer == 1801.0f || m_fInternalRobotTimer == 1802.0f)  || (m_fInternalRobotTimer == 1901.0f || m_fInternalRobotTimer == 1902.0f)))
    //if(RobotIdStrToInt() == 6)
        PrintFvToRobotIdMap(RobotIdStrToInt(), listMapFVsToRobotIds, 9);*/

    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed

#endif
}

/****************************************/
/****************************************/

void SenseCommunicateDetect(std::ofstream &m_cOutput, unsigned RobotId, Real leftSpeed_prev, Real rightSpeed_prev,
                            Real m_fInternalRobotTimer, CCI_EPuckPseudoRangeAndBearingSensor::TPackets &rabsensor_readings,
                            t_listMapFVsToRobotIds& listMapFVsToRobotIds, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listFVsSensed& listFVsSensed,
                            CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                            CRandom::CRNG* m_pcRNG_FVs, unsigned& m_uRobotFV, std::string  &swarmbehav, std::vector<int> &beaconrobots_ids)
{
    unsigned m_uRobotId;


    /****************************************/

    /****************************************/

    /****************************************/
#if FV_MODE == BAYESIANINFERENCE_MODE

    /* Estimating FVs proprioceptively */
    /*encoders give you the speed at the previous tick not current tick */
    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                                 leftSpeed_prev, rightSpeed_prev);

    m_cProprioceptiveFeatureVector.SimulationStep();
    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();


    /* Estimate feature-vectors - via observation */
    m_uRobotId = RobotId; //RobotIdStrToInt();

#ifdef DEBUG_FV_MESSAGES
    std::cout << "Robot Id " << m_uRobotId << " Prop. Feature-vector " << m_uRobotFV << std::endl;
#endif

    /*encoders give you the speed at the previous tick not current tick */
    m_cBayesianInferredFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                                   leftSpeed_prev, rightSpeed_prev);

    m_cBayesianInferredFeatureVector.SimulationStep(swarmbehav, beaconrobots_ids);


#ifdef DEBUG_FV_MESSAGES
    std::cout << "Sense function start " << std::endl;
#endif

    Sense(PROBABILITY_FORGET_FV, RobotId, m_fInternalRobotTimer, rabsensor_readings,
          listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed,
          m_cProprioceptiveFeatureVector, m_cBayesianInferredFeatureVector,
          m_pcRNG_FVs, swarmbehav, beaconrobots_ids);

#ifdef DEBUG_FV_MESSAGES
    std::cout << "Sense function end " << std::endl;
    PrintFvToRobotIdMap(listMapFVsToRobotIds);
    PrintFvDistribution(listFVsSensed);
#endif
#endif

    struct timespec spec;
    time_t* ptr_simple_time; time_t simple_time;
    ptr_simple_time = &simple_time;
    struct tm* ptr_local_time;

    clock_gettime(CLOCK_REALTIME, &spec);
    simple_time = time(ptr_simple_time);
    ptr_local_time = localtime(ptr_simple_time);

#ifdef DEBUG_FV_MESSAGES
    std::cout << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6) << "\t";
    std::cout << "Step: " << m_fInternalRobotTimer << "\t";
    std::cout << "PropAndObsFVs\t";
    std::cout << "RobotId: " << RobotId << "  PropFV: " << m_uRobotFV << "\t";
#endif

    m_cOutput << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6) << "\t";
    m_cOutput << "Step: " << m_fInternalRobotTimer << "\t";
    m_cOutput << "PropAndObsFVs\t";
    m_cOutput << "RobotId: " << RobotId << "  PropFV: " << m_uRobotFV << "\t";

    t_listMapFVsToRobotIds::iterator itd;
#ifdef DEBUG_FV_MESSAGES
    itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        std::cout << "ObservedRobotId: " << itd->uRobotId << "  ObservedRobotFV: " << itd->uFV << "\t";
        ++itd;
    }
    std::cout << std::endl;
#endif

    itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        m_cOutput << "ObservedRobotId: " << itd->uRobotId << "  ObservedRobotFV: " << itd->uFV << "\t";
        ++itd;
    }
    m_cOutput << std::endl;


#ifdef DEBUG_FV_MESSAGES
    std::cout << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6) << "\t";
    std::cout << "Step: " << m_fInternalRobotTimer << "\t";
    std::cout << "DetailedObs\t";
#endif

    m_cOutput << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6) << "\t";
    m_cOutput << "Step: " << m_fInternalRobotTimer << "\t";
    m_cOutput << "DetailedObs\t";

#ifdef DEBUG_FV_MESSAGES
    itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        std::cout << "ObservedRobotId: " << itd->uRobotId << "  ObservedRobotFV: " << itd->uFV << "  FirstTimeObserved: " << itd->fTimeSensed
                  << "  Range: " << itd->uRange << "  NumObsS-M: " << itd->uNumobs_sm << "  NumObsS-nM: " << itd->uNumobs_nsm
                  << "  NumObsM: " << itd->uNumobs_m << "\t";
        ++itd;
    }
    std::cout << std::endl;
#endif

    itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        m_cOutput << "ObservedRobotId: " << itd->uRobotId << "  ObservedRobotFV: " << itd->uFV << "  FirstTimeObserved: " << itd->fTimeSensed
                  << "  Range: " << itd->uRange << "  NumObsS-M: " << itd->uNumobs_sm << "  NumObsS-nM: " << itd->uNumobs_nsm
                  << "  NumObsM: " << itd->uNumobs_m << "\t";
        ++itd;
    }
    m_cOutput << std::endl;

#ifdef DEBUG_FV_MESSAGES
    std::cout << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6) << "\t";
    std::cout << "Step: " << m_fInternalRobotTimer << "\t";
    std::cout << "FVDist\t";
#endif

    m_cOutput << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6) << "\t";
    m_cOutput << "Step: " << m_fInternalRobotTimer << "\t";
    m_cOutput << "FVDist\t";

    t_listFVsSensed::iterator itdist;
#ifdef DEBUG_FV_MESSAGES
    itdist = listFVsSensed.begin();
    while(itdist != listFVsSensed.end())
    {
        std::cout <<  "ObservedFV: " << itdist->uFV << "  NumRobots: " << itdist->fRobots << "\t";
        ++itdist;
    }
    std::cout << std::endl << std::endl;
#endif

    itdist = listFVsSensed.begin();
    while(itdist != listFVsSensed.end())
    {
        m_cOutput <<  "ObservedFV: " << itdist->uFV << "  NumRobots: " << itdist->fRobots << "\t";
        ++itdist;
    }
    m_cOutput << std::endl << std::endl;
    m_cOutput.flush();
}

/****************************************/
/****************************************/
