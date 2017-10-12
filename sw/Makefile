ifeq ($(OS),Windows_NT)
TARGET = eqmax_utils.exe
CC = i686-w64-mingw32-gcc
LINKER = i686-w64-mingw32-g++
RM=del
INC_IUP  = ./lib/iup/win/include
LD_IUP  = ./lib/iup/win
LIBSP = ./lib/libserialport/win
OBJS = main.o
else
TARGET = eqmax_utils
CC = gcc
LINKER = g++
RM=rm
LBITS := $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
INC_IUP  = ./lib/iup/linux/x64/include
LD_IUP  = ./lib/iup/linux/x64
LIBSP = ./lib/libserialport/linux/x64
else
INC_IUP  = ./lib/iup/linux/x86/include
LD_IUP  = ./lib/iup/linux/x86
LIBSP = ./lib/libserialport/linux/x86
endif
OBJS = main.o
endif
C_FLAG = -I. -I$(INC_IUP) -I$(LIBSP) -c -std=c99
L_FLAG = $(LD_IUP)/libiup.a $(LIBSP)/libserialport.a
ifeq ($(OS),Windows_NT)
L_FLAG += -lkernel32 -luser32 -lgdi32 -lwinspool -lcomdlg32 -ladvapi32 -lshell32 -lole32 -loleaut32 -luuid -lcomctl32 -lsetupapi -mwindows -static 
else
ifeq ($(LBITS),64)
L_FLAG += $(shell pkg-config --libs gtk+-3.0 gdk-3.0)
else
L_FLAG += $(shell pkg-config --libs gtk+-2.0 gdk-2.0)
endif
L_FLAG += -lXext -lX11 -lpng -lm -lz -lfontconfig -lfreetype
endif

$(TARGET):$(OBJS)
	$(LINKER) $^ $(L_FLAG) -o $(TARGET)

$(OBJS):%.o:%.c
	$(CC) -c -o $@ $(C_FLAG) $^

clean:
	$(RM) *.o
	$(RM) $(TARGET)
