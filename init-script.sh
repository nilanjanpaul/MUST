# ssh root@node1-1 'apt-get update && apt-get -y install libpugixml-dev libpugixml1'

echo $#
myArg="$@"
i=2
while [ "$i" -le "$#" ]; do
  echo $myArg[$i] 
  i=$(($i + 1))
done


num=`hostname | grep sb3`
if [ "$num" != "" ]
then
echo "This is sandbox 3"
B12="10.13";
echo $B12
fi

num=`hostname | grep grid`
if [ "$num" != "" ]
then
echo "This is grid console"
B12="10.10";
echo $B12
fi

route_cmd='route add -net 192.168.101.102 netmask 255.255.255.255 dev eth1 gw '$B12'.1.2'
echo $route_cmd

exit

# node1-1 - config eth2
ssh root@node1-1 'ifconfig eth2 192.168.10.1 mtu 8000'
ssh root@node1-1 'sysctl -w net.core.rmem_max=50000000'
ssh root@node1-1 'sysctl -w net.core.wmem_max=1048576'

# node1-2 - config eth2
ssh root@node1-2 'ifconfig eth2 192.168.10.1 mtu 8000'
ssh root@node1-2 'sysctl -w net.core.rmem_max=50000000'
ssh root@node1-2 'sysctl -w net.core.wmem_max=1048576'

# node1-1 - update routing
route_cmd='route add -net 192.168.101.102 netmask 255.255.255.255 dev eth1 gw '$B12'.1.2'
ssh root@node1-1 $(route_cmd)

# node1-2 - update preroute 
ssh root@node1-2 'iptables -t nat -A PREROUTING -d 192.168.101.102 -p all -j DNAT --to-destination 192.168.10.2'
ssh root@node1-2 'sysctl net.ipv4.ip_forward=1'
