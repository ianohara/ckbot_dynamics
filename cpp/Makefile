CC=g++
CFLAGS=-I.
LIBRARIES= -lompl -lboost_program_options -lboost_filesystem -ljson

run_sim: run_sim.o ckbot.o ck_ompl.o
	$(CC) -o run_sim run_sim.o ckbot.o ck_ompl.o $(CFLAGS) $(LIBRARIES)

test: test.o ckbot.o ck_ompl.o
	$(CC) -o test test.o ckbot.o ck_ompl.o $(CFLAGS) $(LIBRARIES)


example: example.o ckbot.o ck_ompl.o
	$(CC) -o example ck_ompl.o example.o ckbot.o $(CFLAGS) $(LIBRARIES)

clean:
	rm *.o example ompl_ex
	rm *.out
