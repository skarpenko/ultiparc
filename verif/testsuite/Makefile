# The Ultiparc Project
# Testsuite Makefile

TESTSUITES := \
	basic_tests \
	syslevel_tests


TESTSUITES_CLEAN := $(addsuffix .clean_target,$(TESTSUITES))
override undefine XTENSIVE


.PHONY: all
all: $(TESTSUITES)


.PHONY: help
help:
	@echo "Ultiparc TestSuite"
	@echo "=================="
	@echo "Targets:"
	@echo "  clean                       - clean tests tree;"
	@echo "  runtests                    - run all tests;"
	@echo "  runtests_xt                 - run all tests including extensive tests;"
	@echo "  runtests_{slm|rtl|vlr}[_xt] - run test on a specific target model."


.PHONY: runtests
runtests: runtests_slm runtests_rtl runtests_vlr


.PHONY: runtests_xt
runtests_xt: runtests_slm_xt runtests_rtl_xt runtests_vlr_xt


.PHONY: runtests_slm_xt
runtests_slm_xt: runtests_slm
runtests_slm_xt: XTENSIVE:=xt


.PHONY: runtests_rtl_xt
runtests_rtl_xt: runtests_rtl
runtests_rtl_xt: XTENSIVE:=xt


.PHONY: runtests_vlr_xt
runtests_vlr_xt: runtests_vlr
runtests_vlr_xt: XTENSIVE:=xt


.PHONY: runtests_slm
runtests_slm: $(TESTSUITES) $(ULTIPARC_HOME)/slm/sys_model/sys_model.elf
	@echo
	@echo "Running SLM tests..."
	perl run_tests.pl slm $(XTENSIVE)


.PHONY: runtests_rtl
runtests_rtl: $(TESTSUITES) $(ULTIPARC_HOME)/rtl/tb_sys_top
	@echo
	@echo "Running RTL tests..."
	perl run_tests.pl rtl $(XTENSIVE)


.PHONY: runtests_vlr
runtests_vlr: $(TESTSUITES) $(ULTIPARC_HOME)/slm/verilated/vlsys_model.elf
	@echo
	@echo "Running VLR tests..."
	perl run_tests.pl vlr $(XTENSIVE)


.PHONY: clean
clean: $(TESTSUITES_CLEAN)


.PHONY: $(TESTSUITES)
$(TESTSUITES):
	$(MAKE) -C $@


.PHONY: $(TESTSUITES_CLEAN)
$(TESTSUITES_CLEAN):
	$(MAKE) -C $(@:.clean_target=) clean
