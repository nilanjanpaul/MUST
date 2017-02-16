all : rf_hw_intf sigproc_avg_bw_pwr sigtran sigproc_start_idx

CFLAGS=-O3
HOST="node1-1"


rf_hw_intf : rf_hw_intf.cpp CTimer.cpp CTimer.h CRadio.hpp CDeviceStorage.hpp CSharedMemSimple.hpp
	g++ -c rf_hw_intf.cpp
	g++ -c CTimer.cpp
	g++ -o rf_hw_intf rf_hw_intf.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -llog4cxx -lrt

sigproc_avg_bw_pwr : sigproc.cpp CTimer.cpp CTimer.h CDeviceStorage.hpp CSharedMemSimple.hpp UDPSimple.hpp CWriteOml.h TCPSimple.hpp
	g++ -c sigproc.cpp $(CFLAGS) -D COMPILE_AVG_BW_PWR
	g++ -c CTimer.cpp $(CFLAGS)
	g++ -o sigproc_avg_bw_pwr sigproc.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -lfftw3f -llog4cxx -lrt -lpthread -loml2

sigproc_start_idx : sigproc.cpp CTimer.cpp CTimer.h CDeviceStorage.hpp CSharedMemSimple.hpp UDPSimple.hpp CWriteOml.h TCPSimple.hpp
	g++ -c sigproc.cpp $(CFLAGS) -D COMPILE_SIG_START_IDX
	g++ -c CTimer.cpp $(CFLAGS)
	g++ -o sigproc_start_idx sigproc.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -lfftw3f -llog4cxx -lrt -lpthread -loml2


sigtran : sigtran.cpp CTimer.cpp CTimer.h CDeviceStorage.hpp CSharedMemSimple.hpp UDPSimple.hpp CWriteOml.h TCPSimple.hpp
	g++ -c sigtran.cpp
	g++ -c CTimer.cpp
	g++ -o sigtran sigtran.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -lfftw3f -llog4cxx -lrt -lpthread -loml2

clean :
	rm rf_hw_intf sigproc_avg_bw_pwr sigtran *.o


remote-clean :
	ssh root@$(HOST) 'rm -rf .tmp'

remote-make :
	ssh root@$(HOST) 'mkdir -p .tmp'
	rsync -ptr *      root@$(HOST):/root/.tmp
	ssh root@$(HOST) 'cd .tmp && make'

remote-release :
	ssh root@$(HOST) 'mkdir -p MIMO'
	ssh root@$(HOST) 'cp /root/.tmp/rf_hw_intf /root/MIMO/.'
	ssh root@$(HOST) 'cp /root/.tmp/logconf.prop /root/MIMO/.'
	ssh root@$(HOST) 'cp /root/.tmp/devices.xml /root/MIMO/.'
	ssh root@$(HOST) 'cp /root/.tmp/sigproc_avg_bw_pwr /root/MIMO/.'


remote-run :
	ssh root@$(HOST) 'cd .tmp; ./multi_xml --conf "devices.xml,/devices/active" --rx-only '

remote-install-deps :
	ssh root@$(HOST) 'apt-get update --fix-missing'
	ssh root@$(HOST) 'apt-get -y install libpugixml-dev libpugixml1 liblog4cxx10 liblog4cxx10-dev octave'
#	ssh root@$(HOST) 'apt-get -y install liboml2 liboml2-dev'

test :
	echo $(HOST)
