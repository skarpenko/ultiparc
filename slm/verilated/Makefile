# The Ultiparc Project
# Verilated system model Makefile

TARGET     := vlsys_model.elf
VL_FLAGS   := -Wno-fatal -O3 --exe
TARGET_DEP := main.cxx vl_sys_top.lst


# Enable trace support
ifneq (,$(filter $(TRACE),1 y yes))
VL_FLAGS += --trace
endif


# Main goal
.PHONY: all
all: $(TARGET)
	@echo "Done."


.PHONY: help
help:
	@echo "Ultiparc Verilated system model"
	@echo "==============================="
	@echo "Options:"
	@echo "  TRACE=1  - enable tracing support."


$(TARGET): $(TARGET_DEP)
	@verilator $(VL_FLAGS) -o ../$(TARGET) -f vl_sys_top.lst
	@$(MAKE) -C obj_dir -f Vsys_top.mk


# Do clean
.PHONY: clean
clean:
	@echo "Clean"
	-@rm -f $(TARGET)
	-@rm -fR obj_dir
	-@rm -f vlt_dump.vcd
