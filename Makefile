all : multi_xml

multi_xml : multi_simple.cpp CTimer.cpp CTimer.h CRadio.hpp DeviceStorage.h CWriteOml.h
	g++ -c multi_simple.cpp
	g++ -c CTimer.cpp
	g++ -o multi_xml multi_simple.o CTimer.o -L/usr/local/lib/x86_64-linux-gnu -lboost_program_options -lboost_system -lboost_thread -luhd -lpugixml -lfftw3f -loml2

clean :
	rm multi_xml *.o

remote-clean :
	ssh root@node21-1 'rm -rf .tmp'

remote-make :
	ssh root@node21-1 'mkdir -p .tmp'
	scp -r * root@node21-1:/root/.tmp/.
	ssh root@node21-1 'cd .tmp && make'

remote-run :
	ssh root@node21-1 'cd .tmp; ./multi_xml --conf "devices.xml,/devices/active" --rx-only '
