/**
 * @file <argos3/plugins/robots/e-puck/real_robot/real_epuck_main.cpp>
 *
 * @author Lorenzo Garattoni - <lgaratto@ulb.ac.be>
 * @author Gianpiero Francesca - <gianpiero.francesca@ulb.ac.be>
 */

#include "real_epuck.h"

#include <argos3/plugins/robots/e-puck/real_robot/real_epuck.h>
#include <argos3/core/utility/configuration/command_line_arg_parser.h>

using namespace argos;

int main(int argc, char** argv)
{
    /* The name of the XML config file */
    std::string strConfigFileName;
    /* The id of the controller to use (as specified in the XML config file) */
    std::string strControllerId;

    try
    {
        /*
         * Parse command line
         */
        bool bUsageHelp;
        CCommandLineArgParser cCLAP;
        cCLAP.AddFlag('h',
                      "help",
                      "display this usage information",
                      bUsageHelp);
        cCLAP.AddArgument<std::string>('c',
                                       "config-file",
                                       "the experiment XML configuration file [REQUIRED]",
                                       strConfigFileName);
        cCLAP.AddArgument<std::string>('i',
                                       "id-controller",
                                       "the ID of the controller you want to use [REQUIRED]",
                                       strControllerId);
        cCLAP.Parse(argc, argv);

        if(bUsageHelp) {
            /* if -h or --help specified, print help descr and nothing else */
            cCLAP.PrintUsage(LOG);
            LOG.Flush();
            return EXIT_SUCCESS;
        } else {
            /* if no -h or --help specified */
            /* be sure to have config file set */
            if (strConfigFileName.empty()) {
                THROW_ARGOSEXCEPTION("No config file given, see help (-h or --help)");
            }
            /* AND controller id set */
            if (strControllerId.empty()) {
                THROW_ARGOSEXCEPTION("No controller id given, see help (-h or --help)");
            }
        } /* else all is ok, we can continue */
    }
    catch (CARGoSException& ex) {
        LOGERR << "[FATAL] Error while parsing args"
               << std::endl
               << ex.what()
               << std::endl;
        LOGERR.Flush();
        return EXIT_FAILURE;
    }

    /*
     * TODO: check the size of the base device structs
     */
    LOG << "[INFO] ======== START GL ========== " << std::endl;
    LOG << "[INFO] SENSOR SIZE IS "
        << sizeof (BaseSensorState)
        << std::endl;
    LOG << "[INFO] ACTUATOR SIZE IS "
        << sizeof (BaseActuatorState)
        << std::endl;
    LOG.Flush();

    /*
     * Init the e-puck
     */
    CRealEPuck* pcRealEPuck;
    try
    {
        pcRealEPuck = &CRealEPuck::GetInstance();
#ifdef DEBUG_EPUCK_MESSAGES
        std::cout << "Calling pcRealEPuck->Init" << std::endl;
#endif
        pcRealEPuck->Init(strConfigFileName, strControllerId);
#ifdef DEBUG_EPUCK_MESSAGES
        std::cout << "Called pcRealEPuck->Init" << std::endl;
#endif
    }
    catch (CARGoSException& ex)
    {
        LOGERR << "[FATAL] Error during initialization"
               << std::endl
               << ex.what()
               << std::endl;
        LOGERR.Flush();
        return EXIT_FAILURE;
    }

    /*
     * Main loop, control step execution, sync
     */
    try
    {
#ifdef DEBUG_EPUCK_MESSAGES
        std::cout << "In EPuck Main Loop start " << std::endl;
#endif
        /* be sure to be sync on the ticks before begining the steps */
        pcRealEPuck->SyncControlStep();
#ifdef DEBUG_EPUCK_MESSAGES
        std::cout << "In EPuck Main Loop finished SyncControlStep" << std::endl;
#endif
        while (!pcRealEPuck->IsExperimentFinished() && !pcRealEPuck->bGoHome())
        {
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop About to receive Sensor Data " << std::endl;
#endif
            /* Receive raw data from robot sensors */
            pcRealEPuck->ReceiveSensorData();
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop Received Sensor Data " << std::endl;
#endif
            /* Perform sensor data post-processing */
            pcRealEPuck->UpdateValues();
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop Update values " << std::endl;
#endif
            /* Execute control step only if we can at this time */
            pcRealEPuck->GetController().ControlStep();
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop Ran ControlStep " << std::endl;
#endif
            /* Synchronize the current step on the ticks from xml config file */
            pcRealEPuck->SyncControlStep();
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop Ran SyncControlStep " << std::endl;
#endif
            /* Send data to robot actuators */
            pcRealEPuck->SendActuatorData();
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop Sent Actuator Data " << std::endl;
#endif
            /* Flush the logs */
            LOG.Flush();
            LOGERR.Flush();
#ifdef DEBUG_EPUCK_MESSAGES
            std::cout << "In EPuck Main Loop Finished Flushing " << std::endl;
#endif
        }
    } catch(CARGoSException& ex)
    {
        LOGERR << "[FATAL] Failed during the control steps: "
               << ex.what() << std::endl;
        LOGERR.Flush();
        return EXIT_FAILURE;
    }
    /* Normal ending conditions */
    LOG << "[INFO] Controller terminated" << std::endl;
    LOG.Flush();
    return EXIT_SUCCESS;
}
