all : multi_simple

multi_simple : multi_simple.cpp CTimer.cpp CTimer.h CRadio.hpp DeviceStorage.h CWriteOml.h
	g++ -c multi_simple.cpp
	g++ -c CTimer.cpp
	g++ -o multi_simple multi_simple.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -loml2

clean :
	rm multi_xml *.o

remote-clean :
	ssh root@node3-20 'rm -rf .tmp'

remote-make :
	ssh root@node3-20 'mkdir -p .tmp'
	scp -r * root@node3-20:/root/.tmp/.
	ssh root@node3-20 'cd .tmp && make'

remote-run :
	ssh root@node3-20 'cd .tmp; ./multi_xml --conf "devices.xml,/devices/active" --rx-only '
