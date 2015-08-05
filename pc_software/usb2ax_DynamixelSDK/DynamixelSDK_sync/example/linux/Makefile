TARGET      = SyncRead
SRCS        = SyncRead.c
OBJS        = $(SRCS:.c=.o)
INCLUDEDIRS += -I../../include
LIBDIRS     += -L../../lib
CFLAGS      = $(INCLUDEDIRS) -W -Wall -O2 -std=gnu99
LFLAGS      = $(LIBDIRS) -ldxl

CC = gcc
AR = ar

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $^ -o $@ $(LFLAGS)

$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) -c $^ -o $@

clean:
	rm -f $(OBJS)

mrproper: clean
	rm -f $(TARGET)
