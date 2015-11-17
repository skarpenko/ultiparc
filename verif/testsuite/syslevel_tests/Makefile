# The Ultiparc Project
# System-level testsuite Makefile

# Test programs list
TESTS := \
	timer.elf \
	memcopy.elf \
	matrix_mul.elf \
	fxp_fft.elf \
	fxp_taylor.elf


# Lists of object files
timer.elf := entry.o timer.o
memcopy.elf := entry.o memcopy.o common.o
matrix_mul.elf := entry.o matrix_mul.o
fxp_fft.elf := entry.o fxp_fft.o
fxp_taylor.elf := entry.o fxp_taylor.o


# List of final targets
TARGETS := \
	$(TESTS) \
	$(TESTS:.elf=.bin) \
	$(TESTS:.elf=.hex) \
	$(TESTS:.elf=.dump)


# Include common options
include $(ULTIPARC_HOME)/verif/testsuite/common/common.mk


all: $(TARGETS)


timer.elf: $(timer.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(timer.elf) $(LDFLAGS)


memcopy.elf: $(memcopy.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(memcopy.elf) $(LDFLAGS)


matrix_mul.elf: $(matrix_mul.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(matrix_mul.elf) $(LDFLAGS)


fxp_fft.elf: $(fxp_fft.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(fxp_fft.elf) $(LDFLAGS)


fxp_taylor.elf: $(fxp_taylor.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(fxp_taylor.elf) $(LDFLAGS)


%.bin: %.elf
	$(GCC_PREFIX)objcopy -O binary $< $@


%.hex: %.elf
	$(GCC_PREFIX)objcopy -O verilog $< $@


%.dump: %.elf
	$(GCC_PREFIX)objdump -D $< > $@


%.o: %.S
	$(GCC_PREFIX)gcc $(CFLAGS) $(ASFLAGS) -c $< -o $@


%.o: %.c
	$(GCC_PREFIX)gcc $(CFLAGS) -c $< -o $@


# Clean
.PHONY: clean
clean:
	-rm -f $(TARGETS) $(TESTS:.elf=.o)
	-rm -f $(foreach TARGET, $(TARGETS), $($(TARGET)))
