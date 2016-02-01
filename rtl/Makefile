# The Ultiparc Project
# RTL model Makefile

all: tb_timer

IVFLAGS := -Iinclude


tb_timer: src/interval_timer.v
	iverilog $(IVFLAGS) -s tb_interval_timer src/interval_timer.v src/tb_interval_timer.v -o tb_timer


trace.vcd: tb_timer
	./tb_timer


.PHONY: waveform
waveform: trace.vcd
	gtkwave trace.vcd


PHONY: clean
clean:
	-rm trace.vcd tb_timer
