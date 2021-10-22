#make file - this is a comment section
 
CC=gcc  #compiler
TARGET=main #target file name
 
all:	main.o gyroscope.o heartrate.o
	$(CC) main.c gyroscope.c heartrate.c -o $(TARGET)
 
clean:
	rm *.o $(TARGET)
