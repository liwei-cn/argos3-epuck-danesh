#!/usr/bin/python

import os

lib_files="/usr/arm-linux-gnueabi/usr/lib/argos3/*.so" # comprises the ARGoS core, ARGoS's generic e-puck build, and a variant of Lorenzo's e-puck build for real e-puck robot 
controller_file = "~/lpuck/argos3-epuck/epuck_build/faultdetection/epuck_hom_swarm"
configuration_file = "~/lpuck/argos3-epuck/src/faultdetection/epuck_hom_swarm.argos"

test_controller_file = "~/lpuck/argos3-epuck/epuck_build/testing/test_controller"
test_configuration_file = "~/lpuck/argos3-epuck/src/testing/test_controller.argos"

ipaddr_epucks = []
ipaddr_epucks.append('192.168.1.201')
ipaddr_epucks.append('192.168.1.202')
ipaddr_epucks.append('192.168.1.206')
ipaddr_epucks.append('192.168.1.207')
ipaddr_epucks.append('192.168.1.208')
ipaddr_epucks.append('192.168.1.209')
ipaddr_epucks.append('192.168.1.210')


for ipaddr in ipaddr_epucks:
    print ipaddr
    #x1 = os.system("scp " + lib_files + " root@"+ipaddr+":/root/ARGoS/.")
    #x2 = os.system("scp " + controller_file + " root@"+ipaddr+":~/.")
    x3 = os.system("scp " + configuration_file + " root@"+ipaddr+":~/.")

    #x4 = os.system("scp " + test_controller_file + " root@"+ipaddr+":~/.")
    #x5 = os.system("scp " + test_configuration_file + " root@"+ipaddr+":~/.")

    #if x1 == 0 and x2 == 0 and x3 == 0 and x4 == 0 and x5 == 0:
    if x3 == 0:
        print "Uploaded files to " + ipaddr
    else:
        print "Could not Upload files to " + ipaddr #+ " " + str(x1) + " " + str(x2) + " " + str(x3)

