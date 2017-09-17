
ROOT_PATH = ..

#
#  オブジェクトファイル名の拡張子の設定
#
OBJEXT = exe

#
#  オブジェクトファイル名の定義
#
OBJNAME = adc_test02
ifdef OBJEXT
	OBJFILE = $(OBJNAME).$(OBJEXT)
else
	OBJFILE = $(OBJNAME)
endif

CC  =	arm-none-eabi-gcc
CXX	=	arm-none-eabi-g++
AR  =	arm-none-eabi-ar
NM  =	arm-none-eabi-nm
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

ifeq ($(DBGENV),ROM)
LDSCRIPT = $(ROOT_PATH)/stm32F4_rom.ld
else
LDSCRIPT = $(ROOT_PATH)/stm32F4_ram.ld
endif

COPTS   = -mcpu=cortex-m3 -Wa,--gstabs -mthumb -mthumb-interwork -mlittle-endian -nostdlib -O2 -g -Wall
CDEFS   = 
INCLUDES = -I.

TARGET_DIR = .
UTASK_DIR = $(ROOT_PATH)
vpath %.c $(UTASK_DIR):$(UTASK_DIR)/stdio
vpath %.S $(UTASK_DIR)

COBJS = $(OBJNAME).o

AOBJS = startup_stm32f4xx.o

ifeq ($(DBGENV),ROM)
CDEFS  := $(CDEFS) -DROM_EXEC=1
endif

ifeq ($(DEBUG),1)
COBJS  := $(COBJS) stm32f4xx.o
CDEFS  := $(CDEFS) -DDEBUG
endif

INCLUDES := -I$(UTASK_DIR)/stdio -I$(UTASK_DIR) $(INCLUDES)
CFLAGS = $(COPTS) $(CDEFS) $(INCLUDES)
LDFLAGS := -nostdlib $(LDFLAGS)

ifdef LDSCRIPT
    LDFLAGS := $(LDFLAGS) -T $(LDSCRIPT)
endif

all:	$(OBJFILE)

$(OBJFILE):	$(COBJS) $(AOBJS)
	$(CC)	$(CFLAGS) $(LDFLAGS) -o $(OBJFILE) \
			$(AOBJS) $(COBJS) -lc -lgcc
	$(NM)	$(OBJFILE) > $(OBJNAME).syms
	$(OBJCOPY) -O srec -S $(OBJFILE) $(OBJNAME).srec
	$(OBJDUMP) -t -h $(OBJFILE) > $(OBJNAME).map
	$(OBJDUMP) -lD --disassemble --source $(OBJFILE) > $(OBJNAME).lst

$(COBJS): %.o: %.c
	$(CC) -c $(CFLAGS) $<

$(AOBJS): %.o: %.S
	$(CC) -c $(CFLAGS) $<

# rules

clean:
	rm -f *.o
	rm -f $(OBJFILE) *.syms *.srec *.map *.lst


