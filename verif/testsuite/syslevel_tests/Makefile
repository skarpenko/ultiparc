# The Ultiparc Project
# System-level testsuite Makefile

# Test programs list
TESTS := \
	timer.elf \
	count_intr.elf \
	memcopy.elf \
	matrix_mul.elf \
	fxp_fft.elf \
	fxp_taylor.elf \
	fibonacci.elf \
	fxp_wavelet.elf \
	flt_fft_tmr.elf \
	multitask.elf \
	wavelet_mt.elf


# Lists of object files
timer.elf := entry.o timer.o
count_intr.elf := entry.o count_intr.o
memcopy.elf := entry.o memcopy.o common.o
matrix_mul.elf := entry.o matrix_mul.o
fxp_fft.elf := entry.o fxp_fft.o
fxp_taylor.elf := entry.o fxp_taylor.o
fibonacci.elf := entry.o fibonacci.o common.o
fxp_wavelet.elf := entry.o fxp_wavelet.o common.o
flt_fft_tmr.elf := entry.o flt_fft_tmr.o common.o
multitask.elf := entry.o multitask.o common.o context.o
wavelet_mt.elf := entry.o wavelet_mt.o common.o context.o


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


count_intr.elf: $(count_intr.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(count_intr.elf) $(LDFLAGS)


memcopy.elf: $(memcopy.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(memcopy.elf) $(LDFLAGS)


matrix_mul.elf: $(matrix_mul.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(matrix_mul.elf) $(LDFLAGS)


fxp_fft.elf: $(fxp_fft.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(fxp_fft.elf) $(LDFLAGS)


fxp_taylor.elf: $(fxp_taylor.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(fxp_taylor.elf) $(LDFLAGS)


fibonacci.elf: $(fibonacci.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(fibonacci.elf) $(LDFLAGS)


fxp_wavelet.elf: $(fxp_wavelet.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(fxp_wavelet.elf) $(LDFLAGS)


flt_fft_tmr.elf: $(flt_fft_tmr.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(flt_fft_tmr.elf) -lm $(LDFLAGS)


multitask.elf: $(multitask.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(multitask.elf) -lm $(LDFLAGS)


wavelet_mt.elf: $(wavelet_mt.elf)
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $(wavelet_mt.elf) -lm $(LDFLAGS)


%.bin: %.elf
	$(GCC_PREFIX)objcopy -O binary $< $@


#%.hex: %.elf
#	$(GCC_PREFIX)objcopy -O verilog $< $@


%.hex: %.bin
	perl -e 'print "\@00000000\n"; while(read(STDIN, my $$b, 4)) { printf "%08x\n", unpack "I", $$b; }' < $< > $@


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
