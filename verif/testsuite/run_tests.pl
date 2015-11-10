#!/usr/bin/perl
# The Ultiparc Project
# Test script

use Term::ANSIColor qw(:constants);


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
		name => "Load Store",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/load_store.bin"
	},
	{
		name => "Shifts",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/shifts.bin"
	},
	{
		name => "Multiply and Divide",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/mul_div.bin"
	},
	{
		name => "Jump Register",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/jumpr.bin"
	},
	{
		name => "Set On Less Than",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/set_on_less.bin"
	},
	{
		name => "Integer Overflow Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/int_overflow.bin"
	},
	{
		name => "Reserved Instruction Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/rsvd_instr.bin"
	},
	{
		name => "Timer Interrupt",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/timer.bin"
	},
	{
		name => "Memory Block Copy",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/memcopy.bin"
	},
	{
		name => "Matrix Multiply",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/matrix_mul.bin"
	},
	{
		name => "Fixed-Point FFT",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_fft.bin"
	},
);

my $passed = 0;
my $failed = 0;
my $total = 0;
my $err;

printf "\nRunning testsuite...\n\n";

foreach $test (@tests) {
	++$total;
	my $fmt = "$total. $test->{'name'}...";
	print "$fmt";
	if (length $fmt < $max_name) {
		print " " x ($max_name - length $fmt);
	}
	$err = system("$sys_model_bin -fw_image $test->{'path'} 1>/dev/null 2>/dev/null");
	if ($err == 0) {
		print GREEN, "PASSED\n", RESET;
		++$passed;
	} else {
		print RED, "FAILED\n", RESET;
		++$failed;
	}
}

print "\nSUMMARY:\n";
print "PASSED = $passed\tFAILED = $failed\tTOTAL = $total\n";

# END
