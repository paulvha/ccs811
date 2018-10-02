# makefile for ccs811. October 2018 / paulvha

CC = gcc
DEPS = CCS811.h bcm2835.h twowire.h
OBJ = ccs811.o ccs811_lib.o dylos.o 
LIBS = -lm -ltwowire -lbcm2835 

.cpp.o:: %c $(DEPS)
	$(CC) -Wall -Werror -c -o $@ $<

ccs811 : $(OBJ)
	$(CC) -o $@ $^ $(LIBS)

.PHONY : clean

clean :
	rm ccs811 $(OBJ)
