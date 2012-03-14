CC=g++
CFLAGS=-I.
LIBRARIES= -lompl

example: example.o ckbot.o ck_ompl.o
	$(CC) -o example example.o ckbot.o $(CFLAGS) $(LIBRARIES)

clean:
	rm *.o example
	rm *.out
