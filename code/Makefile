CC=arm-linux-gcc
CFLAGS=-DNO_DEBUG
EXEC=main
OBJS=$(EXEC).o

all: $(EXEC)
$(EXEC): $(OBJS)
	$(CC) -o $@ $^ -lpthread
clean:
	rm -f $(OBJS) $(EXEC)
