/**
 * @file <argos3/plugins/robots/e-puck/real_robot/real_epuck_pseudo_range_and_bearing_sensor.h>
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

#ifndef REAL_EPUCK_PSEUDO_RANGE_AND_BEARING_SENSOR_H
#define REAL_EPUCK_PSEUDO_RANGE_AND_BEARING_SENSOR_H

//#define DEBUG_RAB_MESSAGES

namespace argos
{
   class CRealEPuckPseudoRangeAndBearingSensor;
}

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_pseudo_range_and_bearing_sensor.h>
//#include <argos3/plugins/robots/e-puck/real_robot/real_epuck_serial_sensor.h>
#include <argos3/plugins/robots/e-puck/real_robot/real_epuck_debugmessages.h>
#include <argos3/plugins/robots/e-puck/real_robot/real_epuck_base.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>

#include<unistd.h>
#include<sys/socket.h>
#include<arpa/inet.h>

#include<pthread.h> //for threading the wifi data receiver function

namespace argos
{

   class CRealEPuckPseudoRangeAndBearingSensor : virtual public CCI_EPuckPseudoRangeAndBearingSensor//,
                                                 //virtual public CRealEPuckSerialSensor
   {

   public:

      CRealEPuckPseudoRangeAndBearingSensor();
      virtual ~CRealEPuckPseudoRangeAndBearingSensor();

      virtual void Init(TConfigurationNode& t_node);
      virtual void UpdateValues();

      /**
       * This struct defines a received deserialized range and bearing packet.  It
       * holds the id of the robot, the distance to it in cms (multiplied by 10) and
       * the bearing to it in degrees (multiplied by 10)
       */
      struct RecvDesrzPkt
      {
         /**
          * Id of the robot.
          */
         UInt8 RobotId;

         /**
          * Distance in cm.
          */
         UInt16 Range;

         /**
          * Angle with respect to the robot heading.
          */
         UInt16 Bearing;

         /**
          * Four byte data packet.
          */
         UInt8 Data[4];

         /**
          * Empty constructor
          */
         RecvDesrzPkt() :
            RobotId(255),
            Range(0),
            Bearing(0)
         {
             Data[0] = 0;
             Data[1] = 0;
             Data[2] = 0;
             Data[3] = 0;
         }
      };

   private:

      void Recvfrom_Handler();

      static void *StartThread(void *pt_arg);

      /**
        * Client socket descriptor
        */
      int socket_desc;

      /**
        * Client details
        */
      struct sockaddr_in client;

      /**
        * Details of sender
        */
      struct sockaddr_in from; char fromipaddress[100];

      /**
       * Thread to receive data from tracking server
       */
      pthread_t recvfrom_thread; bool thread_terminate;

      /**
       * Received data packet details
       */
      UInt8* pun_receivedpacket; unsigned un_maxreceivedpacketsize; int in_receivedpacketsize;

      /**
       * Data packet structure for deserialization
       */
      RecvDesrzPkt ms_RecvDesrzPkt;

      /**
       * Buffer storing the data packet
       */
      UInt8* pun_databuffer; int in_receivedpacketinbuffersize;

      /**
       * Address length of sender
       */
      unsigned int un_fromaddresslength;

      /**
       * This mutex protects the concurrent access to the data buffer of received packets (pun_databuffer)
       */
      pthread_mutex_t m_tBufferQueueMutex;

      /**
       * Max. allowable range of sensor
       */
      Real max_range;

      /**
       * Noise on the range of the sensor
       */
      Real noise_std_dev;

      CRandom::CRNG* m_pcRNG;
   };
}

/****************************************/
/****************************************/

//void* Recvfrom_Handler(void *obj)
//{
//   argos::CRealEPuckPseudoRangeAndBearingSensor* m_pRABSensorObj = reinterpret_cast<argos::CRealEPuckPseudoRangeAndBearingSensor*>(obj);
//   while(!m_pRABSensorObj->thread_terminate)
//   {
//       //bzero((UInt8*) pun_receivedpacket, un_maxreceivedpacketsize);
//       m_pRABSensorObj->in_receivedpacketsize = 0;
//       m_pRABSensorObj->in_receivedpacketsize = recvfrom(m_pRABSensorObj->socket_desc, m_pRABSensorObj->pun_receivedpacket, m_pRABSensorObj->un_maxreceivedpacketsize, 0, (struct sockaddr *)&(m_pRABSensorObj->from), &(m_pRABSensorObj->un_fromaddresslength));
//       if(m_pRABSensorObj->in_receivedpacketsize < 0)
//       {
//         THROW_ARGOSEXCEPTION("Error receiving data: " << ::strerror(errno));
//       }
//       else
//       {
//         std::cout << "Received data packet of size " << m_pRABSensorObj->in_receivedpacketsize << " from " << inet_ntop(AF_INET, &(m_pRABSensorObj->from.sin_addr), m_pRABSensorObj->fromipaddress, INET_ADDRSTRLEN) << " port " << ntohs(m_pRABSensorObj->from.sin_port) << std::endl;

//         if((m_pRABSensorObj->in_receivedpacketsize)%3 != 0)
//         {
//           THROW_ARGOSEXCEPTION("Data packet size should be multiple of 3 <robot id, range, bearing>. Packet may be corrupted");
//         }

//         pthread_mutex_lock(&(m_pRABSensorObj->m_tBufferQueueMutex));
//         memcpy(m_pRABSensorObj->pun_databuffer, m_pRABSensorObj->pun_receivedpacket, m_pRABSensorObj->in_receivedpacketsize * sizeof(UInt8));
//         m_pRABSensorObj->in_receivedpacketinbuffersize = m_pRABSensorObj->in_receivedpacketsize
//         pthread_mutex_unlock(&(m_pRABSensorObj->m_tBufferQueueMutex));
//       }
//   }
//   close(m_pRABSensorObj->socket_desc);
//}

/****************************************/
/****************************************/

#endif
