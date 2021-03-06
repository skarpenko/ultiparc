# The Ultiparc Project
# Basic testsuite Makefile

# Test programs list
TESTS := \
	syscall.elf \
	breakpoint.elf \
	ivtbase.elf \
	proc_id.elf \
	status_reg.elf \
	cause_reg.elf \
	bus_err.elf \
	addr_err.elf \
	load_store.elf \
	int_overflow.elf \
	shifts.elf \
	jumpr.elf \
	set_on_less.elf \
	rsvd_instr.elf \
	mul_div.elf \
	hazards.elf \
	tsc_read.elf \
	wait.elf \
	rfe.elf \
	except_ie.elf \
	nor.elf


# List of final targets
TARGETS := \
	$(TESTS) \
	$(TESTS:.elf=.bin) \
	$(TESTS:.elf=.hex) \
	$(TESTS:.elf=.dump)


# Include common options
include $(ULTIPARC_HOME)/verif/testsuite/common/common.mk


# Tell Make to keep object files
.SECONDARY: $(TESTS:.elf=.o)


all: $(TARGETS)


%.elf: %.o
	$(GCC_PREFIX)ld -T $(ULTIPARC_HOME)/verif/testsuite/common/hw.ld -o $@ $< $(LDFLAGS)


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


# Clean
.PHONY: clean
clean:
	-rm -f $(TARGETS) $(TESTS:.elf=.o)
