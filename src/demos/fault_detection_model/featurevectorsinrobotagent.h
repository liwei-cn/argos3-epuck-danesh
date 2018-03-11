#ifndef FEATUREVECTORSINROBOTAGENT_H_
#define FEATUREVECTORSINROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

#include <list>
#include <algorithm>

/******************************************************************************/
/******************************************************************************/

#include <argos3/core/utility/math/vector2.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/******************************************************************************/
/******************************************************************************/


using namespace argos;


/*
 * Stores the distribution of observed Feature vectors - both recent and past (determined by forget probability)
 */
struct StructFVsSensed
{
    unsigned int uFV;
    double fRobots;
    unsigned int uMostWantedState; // 0:  Dont know - no T-cells to make decision or E approx. equal to R (should not occur as T-cells are seeded for APCs where affinity=1)
    // 1:  Attack
    // 2:  Tolerate

    // proportion of the past time-steps when the FV would have been deemed as abnormal
    // double fSuspicious; //we are now going to use a history of previously sensed feature vectors

    StructFVsSensed(unsigned int fv, double density)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = 999;
    }

    StructFVsSensed(unsigned int fv, double density, unsigned int state)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = state;
    }
};

/*
 * Stores the map of observed FVs to robot ids (one to many function map)
 */
struct DetailedInformationFVsSensed
{
    unsigned int uRobotId;

    unsigned int uFV;
    double       fTimeSensed;

    unsigned int uRange, uNumobs_sm, uNumobs_nsm, uNumobs_m;

    double       f_TimesAttacked, f_TimesTolerated; //! we count the number of times the fv uFV is attacked / tolerated according to the CRM. During the last X time-steps, we only send out the vote near the end of the vote compilation period.

    DetailedInformationFVsSensed(unsigned int robotid, double timesensed, unsigned int fv)
    {
        uFV         = fv;
        uRobotId    = robotid;
        fTimeSensed = timesensed;

        uRange = 0u; uNumobs_sm = 0u; uNumobs_nsm = 0u; uNumobs_m = 0u;

        f_TimesAttacked = 0.0f; f_TimesTolerated = 0.0f;
    }

    DetailedInformationFVsSensed(unsigned int robotid, double timesensed, unsigned int fv, unsigned int range, unsigned int numobs_sm, unsigned int numobs_nsm, unsigned int numobs_m)
    {
        uFV         = fv;
        uRobotId    = robotid;
        fTimeSensed = timesensed;

        uRange = range; uNumobs_sm = numobs_sm; uNumobs_nsm = numobs_nsm; uNumobs_m = numobs_m;

        f_TimesAttacked = 0.0f; f_TimesTolerated = 0.0f;
    }


    /*Additional data structure for establishing consensus on id-fv map entries*/
    std::vector<unsigned> vec_ObserverRobotIds;
    std::vector<unsigned> vec_ObservedRobotFVs;
    std::vector<Real>     vec_TimeObserved;

    std::vector<unsigned> vec_ObserverRobotRange;
    std::vector<unsigned> vec_ObserverRobotNumObservations_SM;
    std::vector<unsigned> vec_ObserverRobotNumObservations_nSM;
    std::vector<unsigned> vec_ObserverRobotNumObservations_M;


    DetailedInformationFVsSensed(unsigned int ObserverRobotId, unsigned int ObservedRobotId, double timesensed, unsigned int fv)
    {
        vec_ObserverRobotIds.clear(); vec_ObservedRobotFVs.clear(); vec_TimeObserved.clear();

        uRobotId = ObservedRobotId;
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);
    }

    DetailedInformationFVsSensed(unsigned int ObserverRobotId, unsigned int ObservedRobotId, double timesensed, unsigned int fv,
                                 unsigned int range, unsigned int numobs_sm, unsigned int numobs_nsm, unsigned int numobs_m)
    {
        vec_ObserverRobotIds.clear(); vec_ObservedRobotFVs.clear(); vec_TimeObserved.clear();
        vec_ObserverRobotRange.clear(); vec_ObserverRobotNumObservations_SM.clear(); vec_ObserverRobotNumObservations_nSM.clear();
        vec_ObserverRobotNumObservations_M.clear();

        uRobotId = ObservedRobotId;
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);

        vec_ObserverRobotRange.push_back(range);
        vec_ObserverRobotNumObservations_SM.push_back(numobs_sm);
        vec_ObserverRobotNumObservations_nSM.push_back(numobs_nsm);
        vec_ObserverRobotNumObservations_M.push_back(numobs_m);
    }

    //    void AddNewInformationFVsSensed(unsigned int ObserverRobotId, double timesensed, unsigned int fv)
    //    {
    //        vec_ObserverRobotIds.push_back(ObserverRobotId);
    //        vec_TimeObserved.push_back(timesensed);
    //        vec_ObservedRobotFVs.push_back(fv);
    //    }

    void AddNewInformationFVsSensed(unsigned int ObserverRobotId, double timesensed, unsigned int fv,
                                    unsigned int range, unsigned int numobs_sm, unsigned int numobs_nsm, unsigned int numobs_m)
    {
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);

        vec_ObserverRobotRange.push_back(range);
        vec_ObserverRobotNumObservations_SM.push_back(numobs_sm);
        vec_ObserverRobotNumObservations_nSM.push_back(numobs_nsm);
        vec_ObserverRobotNumObservations_M.push_back(numobs_m);
    }
};


typedef std::list<StructFVsSensed>              t_listFVsSensed;
typedef std::list<DetailedInformationFVsSensed> t_listMapFVsToRobotIds;

/******************************************************************************/
/******************************************************************************/

/*
 *
 *
 */
void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                           CRandom::CRNG *m_pcRNG, Real m_fProbForget);


/*
 *
 *
 */
void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                          unsigned int fv, unsigned robotId, double timesensed,
                          unsigned int range=0, unsigned int numobs_sm=0, unsigned int numobs_nsm=0, unsigned int numobs_m=0);


/*
 * Removes old maps from FVs to Robot Ids
 * f_IdToFV_MaintenanceTime: how long we maintain a particular mapping from fv to robot id
   when do we delete old map entries ? if an entry is older than f_FvToId_MaintenanceTime, we delete it.
 */
void TrimFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, Real f_CurrentRobotTime, Real f_FvToId_MaintenanceTime);

/*
 *
 */
void PrintFvDistribution(t_listFVsSensed &listFVsSensed);

/*
 *
 */
void PrintFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds);

/******************************************************************************/
/******************************************************************************/

#endif
