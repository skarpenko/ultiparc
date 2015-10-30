# The Ultiparc Project
# Testsuite Makefile

TESTSUITES := \
	basic_tests


TESTSUITES_CLEAN := $(addsuffix .clean_target,$(TESTSUITES))


.PHONY: all
all: $(TESTSUITES)


.PHONY: runtests
runtests: $(TESTSUITES)
	perl run_tests.pl


.PHONY: clean
clean: $(TESTSUITES_CLEAN)


.PHONY: $(TESTSUITES)
$(TESTSUITES):
	$(MAKE) -C $@


.PHONY: $(TESTSUITES_CLEAN)
$(TESTSUITES_CLEAN):
	$(MAKE) -C $(@:.clean_target=) clean
