#!/usr/bin/python

#TODO Script uses the KDE's konsole terminal to run robot controller. Update to be geneic to linux platform's associated terminals

import os

controller_name = "epuck_homswarm_controller"
controller_exe_file = "epuck_hom_swarm"
configuration_file = "epuck_hom_swarm.argos"


ipaddr_epucks = []
#ipaddr_epucks.append('192.168.1.201')
ipaddr_epucks.append('192.168.1.202')
#ipaddr_epucks.append('192.168.1.207')
#ipaddr_epucks.append('192.168.1.208')
#ipaddr_epucks.append('192.168.1.210')


for ipaddr in ipaddr_epucks:
    print "Run  " + ipaddr
    command = " \"./" + controller_exe_file + " -c " + configuration_file + " -i " + controller_name + "\""
    print command 
    #x1 = os.system("konsole --hold -e ssh root@"+ipaddr+command)
    ##x1=os.system("konsole -e 'bash -c \"ls -lhR /home; exec bash\"'")

    x1 = os.system("konsole -e ssh root@" + ipaddr + command)


    if x1 == 0:
        print "Opened console to run " + command + " on " + ipaddr
    else:
        print "Could not open console"
        #print "Could not run controller on " + ipaddr + ". Check the controller files exist, and that the linux board is powered on and conntected to the robotlab network"
    
    print "\n"    
