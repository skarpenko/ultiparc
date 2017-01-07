# The Ultiparc Project
# Common Make options

# Cross-compiler prefix
GCC_PREFIX ?= mipsisa32-elf-

# Architecture specific C flags
ARCH_CFLAGS := -mel -mips1 -march=r3000 -msoft-float

# Find libgcc
LIBGCC         := -lgcc
LIBGCC_PATH    := $(dir $(shell $(GCC_PREFIX)gcc $(ARCH_CFLAGS) -print-libgcc-file-name))
LIBGCC_INCLUDE := $(LIBGCC_PATH)/include

# Find C libraries
LIBC_PATH := $(dir $(shell $(GCC_PREFIX)gcc $(ARCH_CFLAGS) -print-file-name=libc.a))

# C flags
CFLAGS := -O2 $(ARCH_CFLAGS)
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
LDFLAGS += -L$(LIBGCC_PATH) -L$(LIBC_PATH) $(LIBGCC)
