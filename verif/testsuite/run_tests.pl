#!/usr/bin/perl
# The Ultiparc Project
# Test script

use Term::ANSIColor qw(:constants);


# Help screen
if(grep(/^help$/, @ARGV)) {
	print "\n";
	print "Ultiparc TestSuite\n";
	print "==================\n";
	print "  slm - Run tests on system-level model (SystemC);\n";
	print "  rtl - Run tests on RTL;\n";
	print "  xt  - Include extensive tests.\n";
	print "\n";
	exit 0;
}


# Config vars
my $sys_model_bin = "$ENV{'ULTIPARC_HOME'}/slm/sys_model/sys_model.elf";
my $sys_model_rtl = "vvp -n $ENV{'ULTIPARC_HOME'}/rtl/tb_sys_top +NOTRACE";
my $max_name = 40;


# Test types
use constant { TEST_BASIC => 0, TEST_SYSLVL => 1, TEST_XTENSIV => 2 };


# Tests list
my @tests = (
	{
		name => "System Call Exception",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/syscall.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/syscall.hex"
	},
	{
		name => "Breakpoint Exception",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/breakpoint.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/breakpoint.hex"
	},
	{
		name => "IVT Base Address Change",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/ivtbase.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/ivtbase.hex"
	},
	{
		name => "Processor Id",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/proc_id.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/proc_id.hex"
	},
	{
		name => "Status Register",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/status_reg.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/status_reg.hex"
	},
	{
		name => "Cause Register",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/cause_reg.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/cause_reg.hex"
	},
	{
		name => "Address Error Exception",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/addr_err.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/addr_err.hex"
	},
	{
		name => "Load Store",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/load_store.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/load_store.hex"
	},
	{
		name => "Shifts",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/shifts.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/shifts.hex"
	},
	{
		name => "Multiply and Divide",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/mul_div.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/mul_div.hex"
	},
	{
		name => "Jump Register",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/jumpr.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/jumpr.hex"
	},
	{
		name => "Set On Less Than",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/set_on_less.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/set_on_less.hex"
	},
	{
		name => "Integer Overflow Exception",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/int_overflow.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/int_overflow.hex"
	},
	{
		name => "Reserved Instruction Exception",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/rsvd_instr.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/rsvd_instr.hex"
	},
	{
		name => "Pipeline hazards",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/hazards.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/hazards.hex"
	},
	{
		name => "Timestamp counter read",
		type => TEST_BASIC,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/tsc_read.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/basic_tests/tsc_read.hex"
	},
	{
		name => "Timer Interrupt",
		type => TEST_SYSLVL,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/timer.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/timer.hex"
	},
	{
		name => "Memory Block Copy",
		type => TEST_SYSLVL,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/memcopy.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/memcopy.hex"
	},
	{
		name => "Matrix Multiply",
		type => TEST_SYSLVL,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/matrix_mul.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/matrix_mul.hex"
	},
	{
		name => "Fixed-Point FFT",
		type => TEST_SYSLVL,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_fft.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_fft.hex"
	},
	{
		name => "Fixed-Point Taylor series",
		type => TEST_SYSLVL,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_taylor.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/fxp_taylor.hex"
	},
	{
		name => "Floating-Point FFT with timer",
		type => TEST_XTENSIVE,
		bin_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/flt_fft_tmr.bin",
		hex_path => "$ENV{'ULTIPARC_HOME'}/verif/testsuite/syslevel_tests/flt_fft_tmr.hex"
	},
);


# Process command line
my ($model) = @ARGV;
my @test_types = ( TEST_BASIC, TEST_SYSLVL );

if (not defined $model) {
	$model = "slm";
} else {
	$model = lc $model;
}

if ($model ne "slm" && $model ne "rtl") {
	$model = "slm";
}

if(grep(/^xt$/, @ARGV)) {
	push @test_types, TEST_XTENSIVE;
}


my $passed = 0;
my $failed = 0;
my $total = 0;
my $err;

printf "\nRunning testsuite...\n\n";

foreach $test (@tests) {
	if(grep(/^$test->{'type'}$/, @test_types)) {
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
