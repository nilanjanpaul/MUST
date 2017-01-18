all : rf_hw_intf #multi_simple 

CFLAGS=-O3

#multi_simple : multi_simple.cpp CTimer.cpp CTimer.h CRadio.hpp DeviceStorage.h CWriteOml.h CSharedMemSimple.hpp
#	g++ -c multi_simple.cpp
#	g++ -c CTimer.cpp
#	g++ -o multi_simple multi_simple.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -loml2 -lrt

rf_hw_intf : rf_hw_intf.cpp CTimer.cpp CTimer.h CRadio.hpp CDeviceStorage.hpp CSharedMemSimple.hpp
	g++ -c rf_hw_intf.cpp
	g++ -c CTimer.cpp
	g++ -o rf_hw_intf rf_hw_intf.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -llog4cxx -lrt

clean :
	rm multi_xml *.o

remote-clean :
	ssh root@node1-1 'rm -rf .tmp'

remote-make :
	ssh root@node1-1 'mkdir -p .tmp'
	rsync -pt * root@node1-1:/root/.tmp
	ssh root@node1-1 'cd .tmp && make'

remote-run :
	ssh root@node1-1 'cd .tmp; ./multi_xml --conf "devices.xml,/devices/active" --rx-only '

remote-install-deps :
	ssh root@node1-1 'apt-get update --fix-missing'
	ssh root@node1-1 'apt-get -y install liboml2 liboml2-dev liblog4cxx10 liblog4cxx10-dev octave'

