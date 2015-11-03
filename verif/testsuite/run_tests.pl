#!/bin/perl
# The Ultiparc Project
# Test script


my $sys_model_bin = "$ENV{'ULTIPARC_HOME'}/slm/sys_model/sys_model.elf";
my $max_name = 40;

# Tests list
my @tests = (
	{
		name => "System Call Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/syscall.bin"
	},
	{
		name => "Breakpoint Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/breakpoint.bin"
	},
	{
		name => "IVT Base Address Change",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/ivtbase.bin"
	},
	{
		name => "Processor Id",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/proc_id.bin"
	},
	{
		name => "Status Register",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/status_reg.bin"
	},
	{
		name => "Address Error Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/addr_err.bin"
	},
	{
		name => "Timer Interrupt",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/timer.bin"
	},
	{
		name => "Memory Block Copy",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/memcopy.bin"
	},
);

my $passed = 0;
my $failed = 0;
my $total = 0;
my $err;

printf "\nRunning testsuite...\n\n";

foreach $test (@tests) {
	++$total;
	print "$total. $test->{'name'}...";
	if (length $test->{'name'} < $max_name) {
		print " " x ($max_name - length $test->{'name'});
	}
	$err = system("$sys_model_bin -fw_image $test->{'path'} 1>/dev/null 2>/dev/null");
	if ($err == 0) {
		print "PASSED\n";
		++$passed;
	} else {
		print "FAILED\n";
		++$failed;
	}
}

print "\nSUMMARY:\n";
print "PASSED = $passed\tFAILED = $failed\tTOTAL = $total\n";

# END
