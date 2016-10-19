# The Ultiparc Project
# Testsuite Makefile

TESTSUITES := \
	basic_tests \
	syslevel_tests


TESTSUITES_CLEAN := $(addsuffix .clean_target,$(TESTSUITES))


.PHONY: all
all: $(TESTSUITES)


.PHONY: runtests
runtests: runtests_slm runtests_rtl


.PHONY: runtests_slm
runtests_slm: $(TESTSUITES)
	@echo
	@echo "Running SLM tests..."
	perl run_tests.pl slm


.PHONY: runtests_rtl
runtests_rtl: $(TESTSUITES)
	@echo
	@echo "Running RTL tests..."
	perl run_tests.pl rtl


.PHONY: clean
clean: $(TESTSUITES_CLEAN)


.PHONY: $(TESTSUITES)
$(TESTSUITES):
	$(MAKE) -C $@


.PHONY: $(TESTSUITES_CLEAN)
$(TESTSUITES_CLEAN):
	$(MAKE) -C $(@:.clean_target=) clean
