
CC = gcc             
CFLAGS = -Wall -lpthread -lrt
SRCS = main.c cleaner.c controller.c initializer.c panel.c sensor.c handleSig.c  
OBJS = $(SRCS:.c=.o)  

EXEC = program

HEADERS = cleaner.h controller.h initializer.h panel.h sensor.h dStruct.h handleSig.h

$(EXEC): $(OBJS)
	$(CC) -o $(EXEC) $(OBJS) $(CFLAGS)

%.o: %.c $(HEADERS)
	$(CC) -c $< $(CFLAGS)

clean:
	rm -f $(OBJS) $(EXEC)


