/**
 * @file <argos3/plugins/robots/e-puck/control_interface/ci_epuck_pseudo_range_and_bearing_sensor.h>
 *
 * @brief This file provides the definition of the epuck pseudo range and bearing
 * sensor.
 *
 * This file provides the definition of the epuck pseudo range and bearing sensor.
 * The range and bearing sensor readings are actually provided by an external overhead camera tracking system.
 * Each e-puck is equipped with a unqiue ArUco fiducial tag used to track the position and orientation of the robot.
 * The tracing server communicates the range and bearing of the neighbouring robots in line-of-sight
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

#ifndef CCI_EPUCK_PSEUDO_RANGE_AND_BEARING_SENSOR_H
#define CCI_EPUCK_PSEUDO_RANGE_AND_BEARING_SENSOR_H

namespace argos
{
   class CCI_EPuckPseudoRangeAndBearingSensor;
}

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/config.h>

namespace argos
{
   class CCI_EPuckPseudoRangeAndBearingSensor : public CCI_Sensor
   {

   public:

       /**
        * This struct defines a received range and bearing packet.  It
        * holds the id of the sending robot, the distance (in cms) and
        * the bearing
        */
       struct SReceivedPacket
       {
          /**
           * Id of the robot.
           */
          UInt8 RobotId;

          /**
           * Distance in cm.
           */
          Real Range;

          /**
           * Angle with respect to the robot heading.
           * The X axis corresponds to the front of the robot;
           * the Y axis corresponds to the left wheel.
           */
          CRadians Bearing;

          /**
           * Four byte data packet
           */
          UInt8 Data[4];

          /**
           * Empty constructor
           */
          SReceivedPacket() :
             RobotId(255),
             Range(0.0f),
             Bearing(0.0)
          {
              Data[0] = 0;
              Data[1] = 0;
              Data[2] = 0;
              Data[3] = 0;
          }


          /**
           * Redefine the "=" operator
           */
          SReceivedPacket& operator=(const SReceivedPacket & t_packet)
          {
             if (&t_packet != this)
             {
                RobotId = t_packet.RobotId;
                Range   = t_packet.Range;
                Bearing = t_packet.Bearing;
                Data[0] = t_packet.Data[0];
                Data[1] = t_packet.Data[1];
                Data[2] = t_packet.Data[2];
                Data[3] = t_packet.Data[3];
             }
             return *this;
          }

          /**
           * Redefine the "<<" operator with a packet
           */
          friend std::ostream& operator<<(std::ostream& os,
                                          const SReceivedPacket & t_packet);
       };

       /**
        * type alias for received packets
        */
       typedef std::vector<SReceivedPacket*> TPackets;

   public:

      /**
       * Constructor
       */
      CCI_EPuckPseudoRangeAndBearingSensor();

      /**
       * Destructor
       */
      virtual ~CCI_EPuckPseudoRangeAndBearingSensor() {}

      void Init(TConfigurationNode& t_node);

      /**
       * Clears the messages received from the range and bearing.
       * Call this at the beginning of your time-step's update function to be sure that at each
       * time-step you only have the most recently received packets.
       */
      void ClearPackets();

      /**
       * Returns the currently stored packets.
       * @return The currently stored packets
       */
      inline const TPackets& GetPackets() const
      {
         return m_tPackets;
      }


#ifdef ARGOS_WITH_LUA
      /**
       *
       * @param pt_lua_state
       */
      virtual void CreateLuaState(lua_State* pt_lua_state);

      /**
       *
       * @param pt_lua_state
       */
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:
      /**
       * Store the readings
       */
      TPackets m_tPackets;
   };
}
#endif
