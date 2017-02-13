#!/usr/bin/python

import os


ipaddr_epucks = []
ipaddr_epucks.append('192.168.1.201')
ipaddr_epucks.append('192.168.1.202')
ipaddr_epucks.append('192.168.1.206')
ipaddr_epucks.append('192.168.1.207')
ipaddr_epucks.append('192.168.1.208')
ipaddr_epucks.append('192.168.1.209')
ipaddr_epucks.append('192.168.1.210')


for ipaddr in ipaddr_epucks:
    print "Power down " + ipaddr
    x1 = os.system("ssh root@"+ipaddr+" poweroff &")

    if x1 == 0:
        print "Powered down linux board at " + ipaddr
    else:
        print "Could not power down linux board at  " + ipaddr + ". The board may already be powered down."
