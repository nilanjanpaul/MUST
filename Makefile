all : multi_xml

multi_xml : multi_simple.cpp CTimer.cpp CTimer.h CRadio.hpp DeviceStorage.h
	g++ -c multi_simple.cpp
	g++ -c CTimer.cpp
	g++ -o multi_xml multi_simple.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f

clean :
	rm multi_xml *.o

remote-clean :
	ssh root@node1-1 'rm -rf .RX2'

remote-make :
	ssh root@node1-1 'mkdir -p .RX2'
	scp -r * root@node1-1:/root/.RX2/.
	ssh root@node1-1 'cd .RX2 && make'

remote-run :
	ssh root@node1-1 'cd .RX2; ./multi_xml --conf "devices.xml,/devices/active" --rx-only '
