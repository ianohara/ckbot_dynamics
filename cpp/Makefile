CC=g++
CFLAGS=-I.
LIBRARIES= -lompl -lboost_program_options -lboost_filesystem -ljson -lboost_system

run_sim: run_sim.o sim_util.o ckbot.o ck_ompl.o
	$(CC) -o run_sim run_sim.o sim_util.o ckbot.o ck_ompl.o $(CFLAGS) $(LIBRARIES)

dynamics: dynamics_verifier.o ckbot.o ck_ompl.o
	$(CC) -o dynamics_verifier dynamics_verifier.o ckbot.o ck_ompl.o $(CFLAGS) $(LIBRARIES)

all:
	run_sim

test: test.o ckbot.o ck_ompl.o
	$(CC) -o test test.o ckbot.o ck_ompl.o $(CFLAGS) $(LIBRARIES)


example: example.o ckbot.o ck_ompl.o
	$(CC) -o example ck_ompl.o example.o ckbot.o $(CFLAGS) $(LIBRARIES)

clean:
	rm *.o run_sim example ompl_ex
	rm *.out
