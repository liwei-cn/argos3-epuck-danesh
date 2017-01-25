/**
 * @file <argos3/plugins/robots/e-puck/control_interface/ci_epuck_pseudo_range_and_bearing_sensor.cpp>
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

#include "ci_epuck_pseudo_range_and_bearing_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos
{

   /****************************************/
   /****************************************/

   CCI_EPuckPseudoRangeAndBearingSensor::CCI_EPuckPseudoRangeAndBearingSensor()
   {
        ClearPackets();
   }

   /****************************************/
   /****************************************/

   void CCI_EPuckPseudoRangeAndBearingSensor::Init(TConfigurationNode& t_node)
   {

   }

   /****************************************/
   /****************************************/

   void CCI_EPuckPseudoRangeAndBearingSensor::ClearPackets()
   {
       while(!m_tPackets.empty())
       {
          delete m_tPackets.back();
          m_tPackets.pop_back();
       }
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuckPseudoRangeAndBearingSensor::CreateLuaState(lua_State* pt_lua_state)
   {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "pseudo_range_and_bearing");
      for(size_t i = 0; i < GetReadings().size(); ++i)
      {
         CLuaUtility::StartTable(pt_lua_state, i+1                               );
         CLuaUtility::AddToTable(pt_lua_state, "robotid",  m_tPackets[i].RobotId);
         CLuaUtility::AddToTable(pt_lua_state, "range",    m_tPackets[i].Range  );
         CLuaUtility::AddToTable(pt_lua_state, "bearing",  m_tPackets[i].Bearing);
         CLuaUtility::EndTable  (pt_lua_state                                    );
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   // todo update this function to also include robotid and bearing in pt_lua_state
   void CCI_EPuckPseudoRangeAndBearingSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "pseudo_range_and_bearing");
      for(size_t i = 0; i < GetReadings().size(); ++i) {
         lua_pushnumber(pt_lua_state, i+1                 );
         lua_gettable  (pt_lua_state, -2                  );
         lua_pushnumber(pt_lua_state, m_tPackets[i].Range);
         lua_setfield  (pt_lua_state, -2, "range"         );
         lua_pop(pt_lua_state, 1);
      }
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

   std::ostream& operator<<(std::ostream& os,
                            const CCI_EPuckPseudoRangeAndBearingSensor::SReceivedPacket & t_packet) {
      os << "PSEUDO_RAB_RECV_DATA < range = " << t_packet.Range
         << ", bearing horizontal = " << t_packet.Bearing
         << ", id = " << t_packet.RobotId;
      return os;
   }

   /****************************************/
   /****************************************/
}
