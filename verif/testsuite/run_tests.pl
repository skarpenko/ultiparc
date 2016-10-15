#!/usr/bin/perl
# The Ultiparc Project
# Test script

use Term::ANSIColor qw(:constants);


my $sys_model_bin = "$ENV{'ULTIPARC_HOME'}/slm/sys_model/sys_model.elf";
my $sys_model_rtl = "vvp -n $ENV{'ULTIPARC_HOME'}/rtl/tb_sys_top +NOTRACE";
my $max_name = 40;


# Tests list
my @tests = (
	{
		name => "System Call Exception",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/syscall.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/syscall.hex"
	},
	{
		name => "Breakpoint Exception",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/breakpoint.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/breakpoint.hex"
	},
	{
		name => "IVT Base Address Change",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/ivtbase.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/ivtbase.hex"
	},
	{
		name => "Processor Id",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/proc_id.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/proc_id.hex"
	},
	{
		name => "Status Register",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/status_reg.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/status_reg.hex"
	},
	{
		name => "Address Error Exception",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/addr_err.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/addr_err.hex"
	},
	{
		name => "Load Store",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/load_store.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/load_store.hex"
	},
	{
		name => "Shifts",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/shifts.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/shifts.hex"
	},
	{
		name => "Multiply and Divide",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/mul_div.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/mul_div.hex"
	},
	{
		name => "Jump Register",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/jumpr.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/jumpr.hex"
	},
	{
		name => "Set On Less Than",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/set_on_less.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/set_on_less.hex"
	},
	{
		name => "Integer Overflow Exception",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/int_overflow.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/int_overflow.hex"
	},
	{
		name => "Reserved Instruction Exception",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/rsvd_instr.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/rsvd_instr.hex"
	},
	{
		name => "Pipeline hazards",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/hazards.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/hazards.hex"
	},
	{
		name => "Timer Interrupt",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/timer.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/timer.hex"
	},
	{
		name => "Memory Block Copy",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/memcopy.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/memcopy.hex"
	},
	{
		name => "Matrix Multiply",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/matrix_mul.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/matrix_mul.hex"
	},
	{
		name => "Fixed-Point FFT",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_fft.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_fft.hex"
	},
	{
		name => "Fixed-Point Taylor series",
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_taylor.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_taylor.hex"
	},
);


# Process command line
my ($model) = @ARGV;

if (not defined $model) {
	$model = "slm";
} else {
	$model = lc $model;
}

if ($model ne "slm" && $model ne "rtl") {
	$model = "slm";
}


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
	if($model eq "slm") {
		$err = run_slm_test("$test->{'bin_path'}");
	} else {
		$err = run_rtl_test("$test->{'hex_path'}");
	}
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


# run SystemC model tests
sub run_slm_test {
	my ($path) = @_;
	my $err = system("$sys_model_bin -fw_image $path 1>/dev/null 2>/dev/null");
	return $err;
}


# run RTL model tests
sub run_rtl_test {
	my ($path) = @_;
	my $log = "sim_result_$$.log";
	my $cmd = "$sys_model_rtl +MEMORY_FILE=$path 1>$log 2>/dev/null";
	my $err = system($cmd);
	if ($err != 0) {
		system("rm $log 1>/dev/null 2>/dev/null");
		return $err;
	}

	open IN, "<$log" or die;
	$err = -1;
	while(my $line = <IN>) {
		chomp $line;
		if($line eq "SIMULATION SUCCESSFULLY TERMINATED!") {
			$err = 0;
			last;
		}
	}
	close IN;

	system("rm $log 1>/dev/null 2>/dev/null");

	return $err;
}


# END
