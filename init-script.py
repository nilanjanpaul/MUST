import sys
import os
import re

#print "total args = ",len(sys.argv) 
if len(sys.argv) < 3:
    print "Usage: python",sys.argv[0]," node1-1 node1-2\n"
    sys.exit();

host = sys.argv[1]
rhost = sys.argv[2:]
print host
print rhost," - ", len(rhost)

cmd = 'hostname'
retvalue = os.popen(cmd).readlines()
if (len(retvalue) == 1 and "sb3" in retvalue[0]):
    console = "sb3";
    B12 = "10.13"
elif (len(retvalue) == 1 and "sb7" in retvalue[0]):
    console = "sb7";
    B12 = "10.17"
elif (len(retvalue) == 1 and "grid" in retvalue[0]):
    console = "grid"
    B12 = "10.10"
else:
    print "Unknown machine"
    sys.exit()





cmd = "ssh -o StrictHostKeyChecking=no root@"+host+" \'ifconfig eth2 192.168.10.1 mtu 8000\'  "; print cmd ; os.popen(cmd)
cmd = "ssh -o StrictHostKeyChecking=no root@"+host+" \'sysctl -w net.core.rmem_max=50000000\' "; print cmd ; os.popen(cmd)
cmd = "ssh -o StrictHostKeyChecking=no root@"+host+" \'sysctl -w net.core.wmem_max=1048576\'  "; print cmd ; os.popen(cmd)
for h in rhost:
    x = h.split('-',1)[0]; x = re.sub('[^0-9]','', x); x100 = str(int(x)+100)
    y = h.split('-',1)[1];                             y100 = str(int(y)+100)
    cmd = "ssh -o StrictHostKeyChecking=no root@"+host+" \'route add -net 192.168."+x100+"."+y100+" netmask 255.255.255.255 dev eth1 gw "+B12+"."+x+"."+y+"\' "; print cmd ; os.popen(cmd)
    

    
for h in rhost:
    x = h.split('-',1)[0]; x = re.sub('[^0-9]','', x); x = str(int(x)+100)
    y = h.split('-',1)[1];                             y = str(int(y)+100)
    #print x," ",y
    cmd = "ssh -o StrictHostKeyChecking=no root@"+h+" \'ifconfig eth2 192.168.10.1 mtu 8000\'  "; print cmd ; os.popen(cmd)
    cmd = "ssh -o StrictHostKeyChecking=no root@"+h+" \'sysctl -w net.core.rmem_max=50000000\' "; print cmd ; os.popen(cmd)
    cmd = "ssh -o StrictHostKeyChecking=no root@"+h+" \'sysctl -w net.core.wmem_max=1048576\'  "; print cmd ; os.popen(cmd)

    cmd = "ssh -o StrictHostKeyChecking=no root@"+h+" \'iptables -t nat -A PREROUTING -d 192.168."+x+"."+y+" -p all -j DNAT --to-destination 192.168.10.2\' "; print cmd ; os.popen(cmd)
    cmd = "ssh -o StrictHostKeyChecking=no root@"+h+" \'sysctl net.ipv4.ip_forward=1\' "; print cmd ; os.popen(cmd)



print "\ndone\n"
