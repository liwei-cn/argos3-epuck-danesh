#include <list>
#include <assert.h>

/******************************************************************************/
/******************************************************************************/

#include "featurevectorsinrobotagent.h"

/******************************************************************************/
/******************************************************************************/


void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                           CRandom::CRNG* m_pcRNG, Real m_fProbForget)
{

    t_listFVsSensed::iterator it_dist, it_history;

    // forget old FVs in distribution with probability m_fProbForgetFV
    it_dist = listFVsSensed.begin();
    while(it_dist != listFVsSensed.end())
    {
        if(m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f)) <= m_fProbForget)
        {
            it_dist = listFVsSensed.erase(it_dist);
            continue;
        }
        ++it_dist;
    }

    t_listFVsSensed tmp_list = t_listFVsSensed(listFVsSensed.begin(), listFVsSensed.end());

    listFVsSensed.clear();

    // updated listFVsSensed distribution with the most recent number of robots for different FVs.
    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
    {
        double increment = 1.0f;

        bool b_EntryInserted(false);

        // check if fv is in listFVsSensed
        // if so, update the value it holds by increment
        // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
        for (it_dist = listFVsSensed.begin(); it_dist != listFVsSensed.end(); ++it_dist)
        {
            if(it_dist->uFV == it_map->uFV)
            {
                // if fv is already present
                it_dist->fRobots += increment;
                b_EntryInserted = true;
                break;
            }

            if(it_dist->uFV > it_map->uFV)
            {   // we assume the list is kept sorted.
                // if fv is absent
                listFVsSensed.insert(it_dist, StructFVsSensed(it_map->uFV, increment));
                b_EntryInserted = true;
                break;
            }
        }

        if(b_EntryInserted)
            continue;

        // when the list is empty or item is to be inserted in the end
        listFVsSensed.push_back(StructFVsSensed(it_map->uFV, increment));
    }

    // integrate into the current list listFVsSensed the history from tmp_list
    // if FV is the same in the current list, and in the history, - the history for that FV is ignored.
    // else the FV is integrated into the current list
    it_dist = listFVsSensed.begin(); it_history = tmp_list.begin();
    while(it_history != tmp_list.end() && it_dist != listFVsSensed.end())
    {
        if(it_history->uFV == it_dist->uFV)
        {
            ++it_history; ++it_dist;
            continue;
        }

        if(it_history->uFV < it_dist->uFV)
        {
                listFVsSensed.insert(it_dist, StructFVsSensed(it_history->uFV, it_history->fRobots, it_history->uMostWantedState));
                ++it_history;
        }
        else
             ++it_dist;
   }

    while(it_history != tmp_list.end())
    {
            listFVsSensed.push_back(StructFVsSensed(it_history->uFV, it_history->fRobots, it_history->uMostWantedState));
            ++it_history;
    }
}

/******************************************************************************/
/******************************************************************************/

void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                          unsigned int fv, unsigned robotId, double timesensed,
                          unsigned int range, unsigned int numobs_sm, unsigned int numobs_nsm, unsigned int numobs_m)
{
    t_listMapFVsToRobotIds::iterator itd;

    // check if robotId is in listMapFVsToRobotIds
    // if so, than update its fv if timesensed is more recent.
    // if not, than add its fv to the list

    bool robot_present(false);
    for (itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
    {
        if((*itd).uRobotId == robotId)
        {
            robot_present = true;

            // if the robot id is already present, add if the new information is more recent
            if(timesensed > (*itd).fTimeSensed)
            {
                (*itd).fTimeSensed = timesensed;
                (*itd).uRobotId = robotId;
                (*itd).uFV = fv;

                (*itd).uRange = range;
                (*itd).uNumobs_sm = numobs_sm;
                (*itd).uNumobs_nsm = numobs_nsm;
                (*itd).uNumobs_m = numobs_m;
            }
            return;
        }
    }

    if(!robot_present) // if the list is empty or the robot with given id is not present in the list
        listMapFVsToRobotIds.push_back(DetailedInformationFVsSensed(robotId, timesensed, fv, range, numobs_sm, numobs_nsm, numobs_m));
}

/******************************************************************************/
/******************************************************************************/

bool sortfventries (const DetailedInformationFVsSensed& i, const DetailedInformationFVsSensed& j) { return (i.fTimeSensed < j.fTimeSensed); }

void TrimFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds, Real f_CurrentRobotTime, Real f_FvToId_MaintenanceTime)
{
#ifndef ConsensusOnMapOfIDtoFV
    // when do we delete entries?
    // if an entry is older than f_FvToId_MaintenanceTime, we delete it.
    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end()) // REMEMBER A FOR LOOP HERE WILL CAUSE RUNTIME ERRORS IF THE LAST ENTRY IN POPULATION IS DELETED.
    {
         if (itd->fTimeSensed < (f_CurrentRobotTime - f_FvToId_MaintenanceTime))
         {
             itd = listMapFVsToRobotIds.erase(itd);
             continue;
         }

         ++itd;
    }
#else
#endif
}

/******************************************************************************/
/******************************************************************************/

void PrintFvDistribution(t_listFVsSensed &listFVsSensed)
{
    std::cout << "Number of entries in FV distribution is " << listFVsSensed.size() << std::endl;

    t_listFVsSensed::iterator itd = listFVsSensed.begin();
    while(itd != listFVsSensed.end())
    {
        std::cout << "FV " << itd->uFV << " number of robots " << itd->fRobots << " state " << itd->uMostWantedState << std::endl;
        ++itd;
    }

    std::cout << "Finished printing FvDistribution" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

void PrintFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds)
{
    std::cout << "Number of entries in detailed map is " << listMapFVsToRobotIds.size() << std::endl;

    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        std::cout << "Observed robot id " << itd->uRobotId << " has fv " << itd->uFV << ", first time sensed is " << itd->fTimeSensed
                  << "range " << itd->uRange << " num s-m obs " << itd->uNumobs_sm << ", num s-nm obs " << itd->uNumobs_nsm
                  << ", num m obs " << itd->uNumobs_m
                  << std::endl;
        ++itd;
    }

    std::cout << "Finished printing FvToRobotIdMap" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

