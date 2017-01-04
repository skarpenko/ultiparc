/*
 * Copyright (c) 2015-2017 The Ultiparc Project. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Main function. Top-level instantiation.
 */

#include <iostream>
#include <iomanip>
#include <string.h>
#include <systemc.h>
#include <sys_top.hxx>
#include <trace.hxx>


sc_trace_file *sys_trace = 0;  // trace file

// MAIN
int sc_main(int argc, char *argv[])
{
	bool do_trace = false;
	const char *firmware_image = 0;

	// Parse command-line arguments
	for(int i=1; i<argc; ++i) {
		if(!strcmp(argv[i], "-trace")) {
			do_trace = true;
		} else if(!strcmp(argv[i], "-fw_image")) {
			++i;
			if(i<argc) {
				firmware_image = argv[i];
			} else {
				std::cerr << "-fw_image: missing file name." << std::endl;
			}
		}
	}

	// Print simulation summary
	std::cout << std::endl;
	std::cout << std::setfill('=') << std::setw(80) << "=" << std::endl;
	std::cout << "Simulation parameters:" << std::endl;
	std::cout << "> Tracing: " << (do_trace ? "ON" : "OFF") << std::endl;
	std::cout << "> Firmware: " << (firmware_image ? firmware_image : "N/A") << std::endl;
	std::cout << std::setfill('=') << std::setw(80) << "=" << std::endl;

	// System clock and reset
	sc_clock sys_clk("sys_clk", 10, SC_NS);
	sc_signal<bool> nrst;

	// To-level
	sys_top top("sys_top");

	// Load memory
	if(firmware_image)
		top.ram.load_memory(firmware_image);

	// Bind signals
	top.clk(sys_clk);
	top.nrst(nrst);

	// Setup tracing
	sys_trace = (do_trace ? sc_create_vcd_trace_file("trace") : 0);
	if(sys_trace) {
		// Clock and reset
		sc_trace_x(sys_trace, sys_clk);
		sc_trace_x(sys_trace, nrst);

		// RAM interface
		sc_trace_x(sys_trace, top.ram.i_MAddr);
		sc_trace_x(sys_trace, top.ram.i_MCmd);
		sc_trace_x(sys_trace, top.ram.i_MData);
		sc_trace_x(sys_trace, top.ram.i_MByteEn);
		sc_trace_x(sys_trace, top.ram.o_SCmdAccept);
		sc_trace_x(sys_trace, top.ram.o_SData);
		sc_trace_x(sys_trace, top.ram.o_SResp);

		// UART interface
		sc_trace_x(sys_trace, top.uart.i_MAddr);
		sc_trace_x(sys_trace, top.uart.i_MCmd);
		sc_trace_x(sys_trace, top.uart.i_MData);
		sc_trace_x(sys_trace, top.uart.i_MByteEn);
		sc_trace_x(sys_trace, top.uart.o_SCmdAccept);
		sc_trace_x(sys_trace, top.uart.o_SData);
		sc_trace_x(sys_trace, top.uart.o_SResp);

		// Simulation control device interface
		sc_trace_x(sys_trace, top.ctrl.i_MAddr);
		sc_trace_x(sys_trace, top.ctrl.i_MCmd);
		sc_trace_x(sys_trace, top.ctrl.i_MData);
		sc_trace_x(sys_trace, top.ctrl.i_MByteEn);
		sc_trace_x(sys_trace, top.ctrl.o_SCmdAccept);
		sc_trace_x(sys_trace, top.ctrl.o_SData);
		sc_trace_x(sys_trace, top.ctrl.o_SResp);

		// CPU instructions fabric port
		sc_trace_x(sys_trace, top.fab.i_cpuI_MAddr);
		sc_trace_x(sys_trace, top.fab.i_cpuI_MCmd);
		sc_trace_x(sys_trace, top.fab.i_cpuI_MData);
		sc_trace_x(sys_trace, top.fab.i_cpuI_MByteEn);
		sc_trace_x(sys_trace, top.fab.o_cpuI_SCmdAccept);
		sc_trace_x(sys_trace, top.fab.o_cpuI_SData);
		sc_trace_x(sys_trace, top.fab.o_cpuI_SResp);

		// CPU data fabric port
		sc_trace_x(sys_trace, top.fab.i_cpuD_MAddr);
		sc_trace_x(sys_trace, top.fab.i_cpuD_MCmd);
		sc_trace_x(sys_trace, top.fab.i_cpuD_MData);
		sc_trace_x(sys_trace, top.fab.i_cpuD_MByteEn);
		sc_trace_x(sys_trace, top.fab.o_cpuD_SCmdAccept);
		sc_trace_x(sys_trace, top.fab.o_cpuD_SData);
		sc_trace_x(sys_trace, top.fab.o_cpuD_SResp);

		// Fabric port 0
		sc_trace_x(sys_trace, top.fab.o_px_MAddr[0]);
		sc_trace_x(sys_trace, top.fab.o_px_MCmd[0]);
		sc_trace_x(sys_trace, top.fab.o_px_MData[0]);
		sc_trace_x(sys_trace, top.fab.o_px_MByteEn[0]);
		sc_trace_x(sys_trace, top.fab.i_px_SCmdAccept[0]);
		sc_trace_x(sys_trace, top.fab.i_px_SData[0]);
		sc_trace_x(sys_trace, top.fab.i_px_SResp[0]);

		// Fabric port 1
		sc_trace_x(sys_trace, top.fab.o_px_MAddr[1]);
		sc_trace_x(sys_trace, top.fab.o_px_MCmd[1]);
		sc_trace_x(sys_trace, top.fab.o_px_MData[1]);
		sc_trace_x(sys_trace, top.fab.o_px_MByteEn[1]);
		sc_trace_x(sys_trace, top.fab.i_px_SCmdAccept[1]);
		sc_trace_x(sys_trace, top.fab.i_px_SData[1]);
		sc_trace_x(sys_trace, top.fab.i_px_SResp[1]);

		// Fabric port 2
		sc_trace_x(sys_trace, top.fab.o_px_MAddr[2]);
		sc_trace_x(sys_trace, top.fab.o_px_MCmd[2]);
		sc_trace_x(sys_trace, top.fab.o_px_MData[2]);
		sc_trace_x(sys_trace, top.fab.o_px_MByteEn[2]);
		sc_trace_x(sys_trace, top.fab.i_px_SCmdAccept[2]);
		sc_trace_x(sys_trace, top.fab.i_px_SData[2]);
		sc_trace_x(sys_trace, top.fab.i_px_SResp[2]);

		// Fabric port 3
		sc_trace_x(sys_trace, top.fab.o_px_MAddr[3]);
		sc_trace_x(sys_trace, top.fab.o_px_MCmd[3]);
		sc_trace_x(sys_trace, top.fab.o_px_MData[3]);
		sc_trace_x(sys_trace, top.fab.o_px_MByteEn[3]);
		sc_trace_x(sys_trace, top.fab.i_px_SCmdAccept[3]);
		sc_trace_x(sys_trace, top.fab.i_px_SData[3]);
		sc_trace_x(sys_trace, top.fab.i_px_SResp[3]);

		// Fabric port 4
		sc_trace_x(sys_trace, top.fab.o_px_MAddr[4]);
		sc_trace_x(sys_trace, top.fab.o_px_MCmd[4]);
		sc_trace_x(sys_trace, top.fab.o_px_MData[4]);
		sc_trace_x(sys_trace, top.fab.o_px_MByteEn[4]);
		sc_trace_x(sys_trace, top.fab.i_px_SCmdAccept[4]);
		sc_trace_x(sys_trace, top.fab.i_px_SData[4]);
		sc_trace_x(sys_trace, top.fab.i_px_SResp[4]);

		// CPU instruction bus
		sc_trace_x(sys_trace, top.cpu.ib_addr_o);
		sc_trace_x(sys_trace, top.cpu.ib_rdc_o);
		sc_trace_x(sys_trace, top.cpu.ib_data_i);
		sc_trace_x(sys_trace, top.cpu.ib_rdy_i);
		sc_trace_x(sys_trace, top.cpu.ib_err_i);

		// CPU data bus
		sc_trace_x(sys_trace, top.cpu.db_addr_o);
		sc_trace_x(sys_trace, top.cpu.db_cmd_o);
		sc_trace_x(sys_trace, top.cpu.db_rnw_o);
		sc_trace_x(sys_trace, top.cpu.db_ben_o);
		sc_trace_x(sys_trace, top.cpu.db_data_o);
		sc_trace_x(sys_trace, top.cpu.db_data_i);
		sc_trace_x(sys_trace, top.cpu.db_rdy_i);
		sc_trace_x(sys_trace, top.cpu.db_err_i);

		// CPU interrupt input
		sc_trace_x(sys_trace, top.cpu.intr_i);

		// CPU registers
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r0_zero);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r1_at);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r2_v0);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r3_v1);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r4_a0);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r5_a1);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r6_a2);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r7_a3);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r8_t0);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r9_t1);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r10_t2);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r11_t3);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r12_t4);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r13_t5);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r14_t6);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r15_t7);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r16_s0);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r17_s1);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r18_s2);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r19_s3);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r20_s4);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r21_s5);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r22_s6);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r23_s7);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r24_t8);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r25_t9);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r26_k0);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r27_k1);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r28_gp);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r29_sp);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r30_s8_fp);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_r31_ra);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_pc);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_hi);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_lo);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_prid);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_epc);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_cause);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_sr);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_psr);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_ivtb);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_tschi);
		sc_trace_x(sys_trace, top.cpu.cpu_reg_tsclo);

		// Interrupt controller
		sc_trace_x(sys_trace, top.intctl.i_MAddr);
		sc_trace_x(sys_trace, top.intctl.i_MCmd);
		sc_trace_x(sys_trace, top.intctl.i_MData);
		sc_trace_x(sys_trace, top.intctl.i_MByteEn);
		sc_trace_x(sys_trace, top.intctl.o_SCmdAccept);
		sc_trace_x(sys_trace, top.intctl.o_SData);
		sc_trace_x(sys_trace, top.intctl.o_SResp);
		sc_trace_x(sys_trace, top.intctl.intr0_i);
		sc_trace_x(sys_trace, top.intctl.cpu_intr_o);

		// Interval timer
		sc_trace_x(sys_trace, top.itimer.i_MAddr);
		sc_trace_x(sys_trace, top.itimer.i_MCmd);
		sc_trace_x(sys_trace, top.itimer.i_MData);
		sc_trace_x(sys_trace, top.itimer.i_MByteEn);
		sc_trace_x(sys_trace, top.itimer.o_SCmdAccept);
		sc_trace_x(sys_trace, top.itimer.o_SData);
		sc_trace_x(sys_trace, top.itimer.o_SResp);
		sc_trace_x(sys_trace, top.itimer.intr_o);
	}

	// Start simulation
	sc_start(0, SC_NS);
	nrst = 0;
	sc_start(100, SC_NS);
	nrst = 1;
	sc_start();

	// Close trace file
	if(sys_trace)
		sc_close_vcd_trace_file(sys_trace);

	// Simulation result
	int err = 0;
	if(top.ctrl.last_value & (1<<31))
		err = -1;

	return err;
}
