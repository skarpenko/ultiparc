# The Ultiparc Project
# Common Make options

# Cross-compiler prefix
GCC_PREFIX ?= mipsisa32-elf-

# Find libgcc
LIBGCC         := -lgcc
LIBGCC_PATH    := $(dir $(shell $(GCC_PREFIX)gcc -print-libgcc-file-name))
LIBGCC_INCLUDE := $(LIBGCC_PATH)/include

# C Flags
CFLAGS := -O2 -mel -mips1 -march=r3000
CFLAGS += -I$(ULTIPARC_HOME)/verif/testsuite/include
CFLAGS += -ffreestanding -fno-pic
CFLAGS += -finline -fno-builtin -fno-strict-aliasing
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -I$(LIBGCC_INCLUDE)
CFLAGS += -Wall

# Assembler flags
ASFLAGS := -D__ASSEMBLY__

# Linker flags
LDFLAGS := --gc-sections
LDFLAGS += -L$(LIBGCC_PATH) -lgcc
