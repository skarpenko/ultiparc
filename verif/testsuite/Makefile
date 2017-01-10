# The Ultiparc Project
# Testsuite Makefile

TESTSUITES := \
	basic_tests \
	syslevel_tests


TESTSUITES_CLEAN := $(addsuffix .clean_target,$(TESTSUITES))
override undefine XTENSIVE


.PHONY: all
all: $(TESTSUITES)


.PHONY: runtests
runtests: runtests_slm runtests_rtl


.PHONY: runtests_xt
runtests_xt: runtests_slm_xt runtests_rtl_xt


.PHONY: runtests_slm_xt
runtests_slm_xt: runtests_slm
runtests_slm_xt: XTENSIVE:=xt


.PHONY: runtests_rtl_xt
runtests_rtl_xt: runtests_rtl
runtests_rtl_xt: XTENSIVE:=xt


.PHONY: runtests_slm
runtests_slm: $(TESTSUITES)
	@echo
	@echo "Running SLM tests..."
	perl run_tests.pl slm $(XTENSIVE)


.PHONY: runtests_rtl
runtests_rtl: $(TESTSUITES)
	@echo
	@echo "Running RTL tests..."
	perl run_tests.pl rtl $(XTENSIVE)


.PHONY: clean
clean: $(TESTSUITES_CLEAN)


.PHONY: $(TESTSUITES)
$(TESTSUITES):
	$(MAKE) -C $@


.PHONY: $(TESTSUITES_CLEAN)
$(TESTSUITES_CLEAN):
	$(MAKE) -C $(@:.clean_target=) clean
