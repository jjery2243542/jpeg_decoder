CC=g++
CFLAGS=-O2 -fopenmp -std=c++14 -g


all:
	$(CC) $(CFLAGS) decoder.cpp -o main
