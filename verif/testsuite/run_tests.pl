#!/bin/perl

# The Ultiparc Project
# Basic testsuite Makefile

my $sys_model_bin = "$ENV{'ULTIPARC_HOME'}/slm/sys_model/sys_model.elf";

my @tests = (
	{
		name => "System Call Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/syscall.bin"
	},
	{
		name => "Breakpoint Exception",
		path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/breakpoint.bin"
	},
);

my $passed = 0;
my $failed = 0;
my $total = 0;
my $err;

printf "\nRunning testsuite...\n\n";

foreach $test (@tests) {
	++$total;
	print "$total. $test->{'name'}...\t\t";
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
