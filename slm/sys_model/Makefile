# The Ultiparc Project
# System model Makefile

CXX_FILES := $(wildcard src/*.cxx)
O_FILES   := $(CXX_FILES:.cxx=.o)
D_FILES   := $(CXX_FILES:.cxx=.d)
TARGET    := sys_model.elf

CFLAGS  := -O3 -g -Wall -Iinclude -I$(SYSTEMC_HOME)/include
LDFLAGS := -static -L$(SYSTEMC_HOME)/lib


# Generete dependency information
# $(call make-depend,source-file,object-file,depend-file)
define make-depend
	gcc -MM \
		-MF $3 \
		-MP \
		-MT $2 \
		$(CFLAGS) \
		$1
endef


# Main goal
.PHONY: all
all: $(TARGET)
	@echo "Done."


$(TARGET): $(O_FILES)
	@echo "Linking [$(TARGET)]"
	@g++ $(LDFLAGS) -o $(TARGET) $(O_FILES) -lsystemc -lpthread


%.o: %.cxx
	@echo "Compiling [$<]"
	@$(call make-depend,$<,$@,$(subst .o,.d,$@))
	@g++ $(CFLAGS) -c $< -o $@


# Load waveform viewer
.PHONY: waveform
waveform: trace.vcd
	gtkwave trace.vcd


# Run simulation
trace.vcd: $(TARGET)
	./$(TARGET) -trace


# Do clean
.PHONY: clean
clean:
	@echo "Clean"
	-@rm -f $(O_FILES)
	-@rm -f $(D_FILES)
	-@rm -f $(TARGET)
	-@rm -f trace.vcd


ifneq "$(MAKECMDGOALS)" "clean"
-include $(D_FILES)
endif
