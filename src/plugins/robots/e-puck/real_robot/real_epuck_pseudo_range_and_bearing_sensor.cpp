/**
 * @file <argos3/plugins/robots/e-puck/real_robot/real_epuck_pseudo_range_and_bearing_sensor.cpp>
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

#include "real_epuck_pseudo_range_and_bearing_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>

static const unsigned int CLIENT_PORT = 8889;

namespace argos
{

    /** time between two receives in micro seconds (so 10 000 micro seconds = 10 milli seconds) */
    static UInt32 TIME_BETWEEN_TWO_RECEIVE = 10000;

   /****************************************/
   /****************************************/

   CRealEPuckPseudoRangeAndBearingSensor::CRealEPuckPseudoRangeAndBearingSensor()
   {
       un_maxreceivedpacketsize = 1024;
       pun_receivedpacket       = new UInt8[un_maxreceivedpacketsize];
       pun_databuffer           = new UInt8[un_maxreceivedpacketsize];
       un_fromaddresslength     = sizeof(struct sockaddr_in);
   }

   /****************************************/
   /****************************************/

   CRealEPuckPseudoRangeAndBearingSensor::~CRealEPuckPseudoRangeAndBearingSensor()
   {
     /*
     * Potential issue:
     * If recv_from blocks and no data is received from server, this function will cause the code to wait indefinitely at pthread_join.
     * A timeout can be placed to prevent the indefinite block along with pthread_cancel(pthread_t*); see example in http://man7.org/linux/man-pages/man3/pthread_cleanup_push.3.html
     */

       thread_terminate = true;

       //Now join the thread; Wait for it to terminate
       if(pthread_join(recvfrom_thread, NULL))
       {
           THROW_ARGOSEXCEPTION("Error joining recvfrom_thread: " << ::strerror(errno));
       }
   }

   /****************************************/
   /****************************************/

   void CRealEPuckPseudoRangeAndBearingSensor::UpdateValues()
   {
       ClearPackets();

       if(in_receivedpacketinbuffersize > 0)
       {
           pthread_mutex_lock(&m_tBufferQueueMutex);
           UInt8* end   = pun_databuffer + in_receivedpacketinbuffersize;
           UInt8* index = pun_databuffer;
           while(index < end)
           {
               memcpy((void*)&(ms_RecvDesrzPkt.RobotId), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);
               memcpy((void*)&(ms_RecvDesrzPkt.Range),   (void*)index, sizeof(UInt16)); index = index + sizeof(UInt16);
               memcpy((void*)&(ms_RecvDesrzPkt.Bearing), (void*)index, sizeof(UInt16)); index = index + sizeof(UInt16);

               SReceivedPacket* sNewPacket = new SReceivedPacket();
               sNewPacket->RobotId         = ms_RecvDesrzPkt.RobotId;
               sNewPacket->Range           = ((Real)ms_RecvDesrzPkt.Range)/10.0f;
               sNewPacket->Bearing.FromValueInDegrees(((Real)ms_RecvDesrzPkt.Bearing)/10.0f);

               m_tPackets.push_back(sNewPacket);
           }

           in_receivedpacketinbuffersize = 0; // to prevent an old packet in buffer from being read again
           pthread_mutex_unlock(&m_tBufferQueueMutex);
       }
   }

   /****************************************/
   /****************************************/

   void CRealEPuckPseudoRangeAndBearingSensor::Init(TConfigurationNode& t_node)
   {
       //Create socket
       socket_desc = socket(AF_INET , SOCK_DGRAM , 0);
       if(socket_desc == -1)
       {
          THROW_ARGOSEXCEPTION("Could not create UDP socket: " << ::strerror(errno));
       }

       //Prepare the sockaddr_in structure for binding
       bzero((char *) &client, sizeof(client));
       client.sin_family = AF_INET;
       client.sin_addr.s_addr = INADDR_ANY;
       client.sin_port = htons(CLIENT_PORT);

       //Bind
       if(bind(socket_desc,(struct sockaddr *)&client , sizeof(client)) < 0)
       {
           THROW_ARGOSEXCEPTION("Binding to port failed: " << ::strerror(errno));
       }
       std::cout << "bind done"  << std::endl;


       // Initialize the mutex
       pthread_mutex_init(&m_tBufferQueueMutex, NULL);

       thread_terminate = false;

       // Start the thread to receive data from server
       if(pthread_create(&recvfrom_thread, NULL, &StartThread, this) < 0)
       {
           THROW_ARGOSEXCEPTION("Could not create recvfrom thread: " << ::strerror(errno));
       }
   }

    /****************************************/
    /****************************************/

   void* CRealEPuckPseudoRangeAndBearingSensor::StartThread(void* pt_arg)
   {
       reinterpret_cast<CRealEPuckPseudoRangeAndBearingSensor*>(pt_arg)->Recvfrom_Handler();
       return NULL;
   }

   /****************************************/
   /****************************************/

    void CRealEPuckPseudoRangeAndBearingSensor::Recvfrom_Handler()
    {
       // If the tracking server is bombarding us with readings, it may be sensible to pace receiving readings  (see CRealEPuckVirtualCamrabSensor::DataFetcherThread())
       while(!thread_terminate)
       {
           //bzero((UInt8*) pun_receivedpacket, un_maxreceivedpacketsize);
           in_receivedpacketsize = 0;
           in_receivedpacketsize = recvfrom(socket_desc, pun_receivedpacket, un_maxreceivedpacketsize, 0, (struct sockaddr *)&from, &un_fromaddresslength);
           if(in_receivedpacketsize < 0)
           {
             THROW_ARGOSEXCEPTION("Error receiving data: " << ::strerror(errno));
           }
           else
           {
             std::cout << "Received data packet of size " << in_receivedpacketsize << " from " << inet_ntop(AF_INET, &(from.sin_addr), fromipaddress, INET_ADDRSTRLEN) << " port " << ntohs(from.sin_port) << std::endl;

             if(in_receivedpacketsize%5 != 0)
             {
               THROW_ARGOSEXCEPTION("Data packet size should be multiple of 5 <robot id, range, bearing>. Packet may be corrupted");
             }

             pthread_mutex_lock(&m_tBufferQueueMutex);
             memcpy(pun_databuffer, pun_receivedpacket, in_receivedpacketsize * sizeof(UInt8));
             in_receivedpacketinbuffersize = in_receivedpacketsize;
             pthread_mutex_unlock(&m_tBufferQueueMutex);
           }

           /*
             * Sleep for a while to avoid the robot being overloaded with range-and-bearing data packets
             * The value is set in microseconds:
             * 10.000 = 10 millisec => 10 times per control step
             */
           usleep(TIME_BETWEEN_TWO_RECEIVE);
       }
       close(socket_desc);
    }

    /****************************************/
    /****************************************/
}

