# The Ultiparc Project
# Common Make options

# Cross-compiler prefix
GCC_PREFIX ?= mipsisa32-elf-

# C Flags
CFLAGS := -mel -mips1 -march=r3000
CFLAGS += -I$(ULTIPARC_HOME)/verif/testsuite/include

# Assembler flags
ASFLAGS := -D__ASSEMBLY__
