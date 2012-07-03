CC=g++
CFLAGS=-I.
LIBRARIES= -lompl -lboost_program_options -lboost_filesystem -ljson -lboost_system

runner: cxxpre runner.o sup_comp
	$(CC) -o runner runner.o sim_util.o ckbot.o ck_ompl.o util.o $(CFLAGS) $(LIBRARIES)

cxxpre:
	./cxxtest/bin/cxxtestgen --error-printer -o runner.cpp ckbot_tests.hpp chain_tests.hpp

sup_comp:
	$(CC) -c ../sim_util.cpp ../ckbot.cpp ../ck_ompl.cpp ../util.cpp

clean:
	rm *.o
