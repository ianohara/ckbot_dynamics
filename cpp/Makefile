CC=g++
CFLAGS=-I.
LIBRARIES= -lompl -lboost_program_options -lboost_filesystem -ljson -lboost_system

run_sim: run_sim.o sim_util.o ckbot.o ck_ompl.o world.o
	$(CC) -o run_sim run_sim.o sim_util.o ckbot.o ck_ompl.o world.o $(CFLAGS) $(LIBRARIES)

dynamics: dynamics_verifier.o ckbot.o ck_odeint.o sim_util.o ck_ompl.o util.o world.o
	$(CC) -o dynamics_verifier dynamics_verifier.o ckbot.o ck_odeint.o sim_util.o util.o ck_ompl.o world.o $(CFLAGS) $(LIBRARIES)

all:
	run_sim
	dynamics

clean:
	rm *.o run_sim dynamics_verifier
	rm *.out
