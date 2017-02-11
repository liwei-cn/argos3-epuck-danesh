/**
 * @file <argos3/plugins/robots/e-puck/real_robot/real_epuck_pseudo_range_and_bearing_sensor.cpp>
 *
 * @author Danesh Tarapore - <danesh.tarapore@york.ac.uk>
 */

#include "real_epuck_pseudo_range_and_bearing_sensor.h"
#include <argos3/core/utility/logging/argos_log.h>

static const unsigned int CLIENT_PORT = 9000;

namespace argos
{

/** time between two receives in micro seconds (so 10 000 micro seconds = 10 milli seconds) */
static UInt32 TIME_BETWEEN_TWO_RECEIVE = 20000;

/****************************************/
/****************************************/

CRealEPuckPseudoRangeAndBearingSensor::CRealEPuckPseudoRangeAndBearingSensor()
{
    m_pcRNG                  = NULL;
    un_maxreceivedpacketsize = 1024;
    pun_receivedpacket       = new UInt8[un_maxreceivedpacketsize];
    pun_databuffer           = new UInt8[un_maxreceivedpacketsize];
    un_fromaddresslength     = sizeof(struct sockaddr_in);

    in_receivedpacketinbuffersize = 0;
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
#ifdef DEBUG_RAB_MESSAGES
    std::cout << "In RAB UpdateValues" << std::endl;
#endif
    ClearPackets();
#ifdef DEBUG_RAB_MESSAGES
    std::cout << "Finished ClearPackets" << std::endl;
#endif

    if(in_receivedpacketinbuffersize > 0)
    {
#ifdef DEBUG_RAB_MESSAGES
        std::cout << "in_receivedpacketinbuffersize " << in_receivedpacketinbuffersize << " Asking for lock m_tBufferQueueMutex" << std::endl;
#endif
        pthread_mutex_lock(&m_tBufferQueueMutex);
#ifdef DEBUG_RAB_MESSAGES
        std::cout << "Lock in function UpdateValues " << std::endl;
#endif
        UInt8* end   = pun_databuffer + in_receivedpacketinbuffersize;
        UInt8* index = pun_databuffer;
        while(index < end)
        {
            memcpy((void*)&(ms_RecvDesrzPkt.RobotId), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);
            memcpy((void*)&(ms_RecvDesrzPkt.Range),   (void*)index, sizeof(UInt16)); index = index + sizeof(UInt16);
            memcpy((void*)&(ms_RecvDesrzPkt.Bearing), (void*)index, sizeof(UInt16)); index = index + sizeof(UInt16);
            memcpy((void*)&(ms_RecvDesrzPkt.Data[0]), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);
            memcpy((void*)&(ms_RecvDesrzPkt.Data[1]), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);
            memcpy((void*)&(ms_RecvDesrzPkt.Data[2]), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);
            memcpy((void*)&(ms_RecvDesrzPkt.Data[3]), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);

            SReceivedPacket* sNewPacket = new SReceivedPacket();
            sNewPacket->RobotId         = ms_RecvDesrzPkt.RobotId;
            sNewPacket->Range           = ((Real)ms_RecvDesrzPkt.Range)/10.0f;
            //sNewPacket->Bearing.FromValueInDegrees(((Real)ms_RecvDesrzPkt.Bearing)/10.0f);
            sNewPacket->Bearing.SetValue( (( ((Real)ms_RecvDesrzPkt.Bearing)/10.0f) - 180.0f) * CRadians::PI.GetValue() / 180.0f);
            sNewPacket->Data[0]         = ms_RecvDesrzPkt.Data[0];
            sNewPacket->Data[1]         = ms_RecvDesrzPkt.Data[1];
            sNewPacket->Data[2]         = ms_RecvDesrzPkt.Data[2];
            sNewPacket->Data[3]         = ms_RecvDesrzPkt.Data[3];

            sNewPacket->Range = std::min(sNewPacket->Range, max_range); // the range can not exceed max_range

            /**
             * Adding noise to the RAB sensor. Disabled for now as the tracking is inherently noisy.
            CVector2 noise_vec(m_pcRNG->Gaussian(noise_std_dev), m_pcRNG->Uniform(CRadians::SIGNED_RANGE));
            CVector2 rab_reading = CVector2(sNewPacket->Range, sNewPacket->Bearing) + noise_vec;
            sNewPacket->Range    = rab_reading.Length();
            sNewPacket->Bearing  = rab_reading.Angle();
            */

            m_tPackets.push_back(sNewPacket);
        }

        in_receivedpacketinbuffersize = 0; // to prevent an old packet in buffer from being read again
        pthread_mutex_unlock(&m_tBufferQueueMutex);
#ifdef DEBUG_RAB_MESSAGES
        std::cout << "Unlock in function UpdateValues " << std::endl;
#endif
    }
}

/****************************************/
/****************************************/

void CRealEPuckPseudoRangeAndBearingSensor::Init(TConfigurationNode& t_node)
{
    m_pcRNG = CRandom::CreateRNG("argos");

    /* get the max range value (in cm) from the configuration file*/
    try
    {
        GetNodeAttributeOrDefault(t_node, "range", max_range, 100.0f);
#ifdef DEBUG_RAB_MESSAGES
        std::cout << "Range set to " << max_range << std::endl;
#endif
    }
    catch (CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Unable to init range attribute "
                                    << "is missing, mandatory", ex);
    }

    try
    {
        GetNodeAttributeOrDefault(t_node, "noise_std_dev", noise_std_dev, 0.0f);
#ifdef DEBUG_RAB_MESSAGES
        std::cout << "noise_std_dev set to " << noise_std_dev << std::endl;
#endif
    }
    catch (CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Unable to init noise_std_dev attribute "
                                    << "is missing, mandatory", ex);
    }



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
#ifdef DEBUG_RAB_MESSAGES
    std::cout << "bind done"  << std::endl;
#endif

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
#ifdef DEBUG_RAB_MESSAGES
            std::cout << "Received data packet of size " << in_receivedpacketsize << " from " << inet_ntop(AF_INET, &(from.sin_addr), fromipaddress, INET_ADDRSTRLEN) << " port " << ntohs(from.sin_port) << std::endl;
#endif
            if(in_receivedpacketsize%9 != 0)
            {
                THROW_ARGOSEXCEPTION("Data packet size should be multiple of 5 <robot id, range, bearing>. Packet may be corrupted");
            }

            pthread_mutex_lock(&m_tBufferQueueMutex);

#ifdef DEBUG_RAB_MESSAGES
            std::cout << "Lock in function Recvfrom_Handler " << std::endl;
#endif

            memcpy(pun_databuffer, pun_receivedpacket, in_receivedpacketsize * sizeof(UInt8));
            in_receivedpacketinbuffersize = in_receivedpacketsize;
            pthread_mutex_unlock(&m_tBufferQueueMutex);

#ifdef DEBUG_RAB_MESSAGES
            std::cout << "Unlock in function Recvfrom_Handler " << std::endl;
#endif
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

