all : rf_hw_intf sigproc #multi_simple 

CFLAGS=-O3
HOST="node1-1"


#multi_simple : multi_simple.cpp CTimer.cpp CTimer.h CRadio.hpp DeviceStorage.h CWriteOml.h CSharedMemSimple.hpp
#	g++ -c multi_simple.cpp
#	g++ -c CTimer.cpp
#	g++ -o multi_simple multi_simple.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -loml2 -lrt

rf_hw_intf : rf_hw_intf.cpp CTimer.cpp CTimer.h CRadio.hpp CDeviceStorage.hpp CSharedMemSimple.hpp
	g++ -c rf_hw_intf.cpp
	g++ -c CTimer.cpp
	g++ -o rf_hw_intf rf_hw_intf.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -llog4cxx -lrt

sigproc : sigproc.cpp CTimer.cpp CTimer.h CRadio.hpp CDeviceStorage.hpp CSharedMemSimple.hpp UDPSimple.hpp CWriteOml.h TCPSimple.hpp
	g++ -c sigproc.cpp $(CFLAGS)
	g++ -c CTimer.cpp $(CFLAGS)
	g++ -o sigproc sigproc.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -lfftw3f -llog4cxx -lrt -lpthread -loml2

clean :
	rm multi_xml *.o

remote-clean :
	ssh root@$(HOST) 'rm -rf .tmp'

remote-make :
	ssh root@$(HOST) 'mkdir -p .tmp'
	rsync -pt * root@$(HOST):/root/.tmp
	ssh root@$(HOST) 'cd .tmp && make'

remote-run :
	ssh root@$(HOST) 'cd .tmp; ./multi_xml --conf "devices.xml,/devices/active" --rx-only '

remote-install-deps :
	ssh root@$(HOST) 'apt-get update --fix-missing'
	ssh root@$(HOST) 'apt-get -y install libpugixml-dev libpugixml1 liboml2 liboml2-dev liblog4cxx10 liblog4cxx10-dev octave'

test :
	echo $(HOST)
