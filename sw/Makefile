TARGET = eqmax_utils
I_DIR = ./lib/linux/x64/include
L_DIR = ./lib/linux/x64
L_FLAG = -L$(L_DIR) -lm -liup
SERIAL_OBJ = linux.o linux_termios.o serialport.o
SERIAL_FLAG = -DHAVE_LIBUDEV
CC = gcc

$(TARGET):main.c $(SERIAL_OBJ)
	$(CC) $^ -finput-charset=cp936 -I$(I_DIR) $(L_FLAG) -o $(TARGET)

linux_termios.o:linux_termios.c linux_termios.h
	$(CC) -c $^

linux.o:linux.c
	$(CC) -c $^

serialport.o:serialport.c libserialport.h
	$(CC) $(SERIAL_FLAG) -c $^

clean:
	rm *.o
	rm $(TARGET)
