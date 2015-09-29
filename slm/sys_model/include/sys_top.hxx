/*
 * Copyright (c) 2015 The Ultiparc Project. All rights reserved.
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
 * System top
 */

#include <systemc.h>
#include "ocp_memory.hxx"
#include "ocp_muart.hxx"
#include "ocp_fabric.hxx"
#include "ibus_adapt.hxx"
#include "dbus_adapt.hxx"
#include "cpu/cpu_top.hxx"
#pragma once


// RAM size to instantiate in the system
#define RAM_SIZE (4*1024*1024)


// System top module
SC_MODULE(sys_top) {
	sc_in<bool> clk;
	sc_in<bool> nrst;


	// System components
	memory     ram;
	fabric     fab;
	muart      uart;
	cpu_top    cpu;
	ibus_adapt ibusa;
	dbus_adapt dbusa;


	SC_CTOR(sys_top)
		: clk("clk"), nrst("nrst"), ram("ram", RAM_SIZE), fab("fab"), uart("uart"),
		  cpu("cpu"), ibusa("ibusa"), dbusa("dbusa")
	{
		// connect clock
		fab.clk(clk);
		ram.clk(clk);
		uart.clk(clk);
		cpu.clk(clk);
		ibusa.clk(clk);
		dbusa.clk(clk);

		// connect reset
		fab.nrst(nrst);
		ram.nrst(nrst);
		uart.nrst(nrst);
		cpu.nrst(nrst);
		ibusa.nrst(nrst);
		dbusa.nrst(nrst);

		// RAM
		ram.i_MAddr(sig_p0_MAddr);
		fab.o_px_MAddr[0](sig_p0_MAddr);
		ram.i_MCmd(sig_p0_MCmd);
		fab.o_px_MCmd[0](sig_p0_MCmd);
		ram.i_MData(sig_p0_MData);
		fab.o_px_MData[0](sig_p0_MData);
		ram.i_MByteEn(sig_p0_MByteEn);
		fab.o_px_MByteEn[0](sig_p0_MByteEn);
		ram.o_SCmdAccept(sig_p0_SCmdAccept);
		fab.i_px_SCmdAccept[0](sig_p0_SCmdAccept);
		ram.o_SData(sig_p0_SData);
		fab.i_px_SData[0](sig_p0_SData);
		ram.o_SResp(sig_p0_SResp);
		fab.i_px_SResp[0](sig_p0_SResp);

		// UART
		uart.i_MAddr(sig_p1_MAddr);
		fab.o_px_MAddr[1](sig_p1_MAddr);
		uart.i_MCmd(sig_p1_MCmd);
		fab.o_px_MCmd[1](sig_p1_MCmd);
		uart.i_MData(sig_p1_MData);
		fab.o_px_MData[1](sig_p1_MData);
		uart.i_MByteEn(sig_p1_MByteEn);
		fab.o_px_MByteEn[1](sig_p1_MByteEn);
		uart.o_SCmdAccept(sig_p1_SCmdAccept);
		fab.i_px_SCmdAccept[1](sig_p1_SCmdAccept);
		uart.o_SData(sig_p1_SData);
		fab.i_px_SData[1](sig_p1_SData);
		uart.o_SResp(sig_p1_SResp);
		fab.i_px_SResp[1](sig_p1_SResp);

		// I-Bus adapter
		fab.i_cpuI_MAddr(sig_cpuI_MAddr);
		ibusa.o_MAddr(sig_cpuI_MAddr);
		fab.i_cpuI_MCmd(sig_cpuI_MCmd);
		ibusa.o_MCmd(sig_cpuI_MCmd);
		fab.i_cpuI_MData(sig_cpuI_MData);
		ibusa.o_MData(sig_cpuI_MData);
		fab.i_cpuI_MByteEn(sig_cpuI_MByteEn);
		ibusa.o_MByteEn(sig_cpuI_MByteEn);
		fab.o_cpuI_SCmdAccept(sig_cpuI_SCmdAccept);
		ibusa.i_SCmdAccept(sig_cpuI_SCmdAccept);
		fab.o_cpuI_SData(sig_cpuI_SData);
		ibusa.i_SData(sig_cpuI_SData);
		fab.o_cpuI_SResp(sig_cpuI_SResp);
		ibusa.i_SResp(sig_cpuI_SResp);

		// D-Bus adapter
		fab.i_cpuD_MAddr(sig_cpuD_MAddr);
		dbusa.o_MAddr(sig_cpuD_MAddr);
		fab.i_cpuD_MCmd(sig_cpuD_MCmd);
		dbusa.o_MCmd(sig_cpuD_MCmd);
		fab.i_cpuD_MData(sig_cpuD_MData);
		dbusa.o_MData(sig_cpuD_MData);
		fab.i_cpuD_MByteEn(sig_cpuD_MByteEn);
		dbusa.o_MByteEn(sig_cpuD_MByteEn);
		fab.o_cpuD_SCmdAccept(sig_cpuD_SCmdAccept);
		dbusa.i_SCmdAccept(sig_cpuD_SCmdAccept);
		fab.o_cpuD_SData(sig_cpuD_SData);
		dbusa.i_SData(sig_cpuD_SData);
		fab.o_cpuD_SResp(sig_cpuD_SResp);
		dbusa.i_SResp(sig_cpuD_SResp);

		// CPU I-Bus
		cpu.ib_addr_o(sig_cpu_ib_addr);
		ibusa.addr_i(sig_cpu_ib_addr);
		cpu.ib_rdc_o(sig_cpu_ib_rdc);
		ibusa.rdc_i(sig_cpu_ib_rdc);
		cpu.ib_data_i(sig_cpu_ib_data);
		ibusa.data_o(sig_cpu_ib_data);
		cpu.ib_rdy_i(sig_cpu_ib_rdy);
		ibusa.rdy_o(sig_cpu_ib_rdy);
		cpu.ib_err_i(sig_cpu_ib_err);
		ibusa.err_o(sig_cpu_ib_err);

		// CPU D-Bus
		cpu.db_addr_o(sig_cpu_db_addr);
		dbusa.addr_i(sig_cpu_db_addr);
		cpu.db_cmd_o(sig_cpu_db_cmd);
		dbusa.cmd_i(sig_cpu_db_cmd);
		cpu.db_rnw_o(sig_cpu_db_rnw);
		dbusa.rnw_i(sig_cpu_db_rnw);
		cpu.db_ben_o(sig_cpu_db_ben);
		dbusa.ben_i(sig_cpu_db_ben);
		cpu.db_data_o(sig_cpu_db_mdata);
		dbusa.data_i(sig_cpu_db_mdata);
		cpu.db_data_i(sig_cpu_db_sdata);
		dbusa.data_o(sig_cpu_db_sdata);
		cpu.db_rdy_i(sig_cpu_db_rdy);
		dbusa.rdy_o(sig_cpu_db_rdy);
		cpu.db_err_i(sig_cpu_db_err);
		dbusa.err_o(sig_cpu_db_err);
	}

private:
#if 0
	void mem_test(void)
	{
		unsigned ia = 0x1000;
		unsigned da = 0x2000;

		wait(nrst.posedge_event());

		while(true) {
			wait();
//cout << sig_p0_MAddr.name() << endl;
			sig_cpuI_MAddr.write(ia);
			sig_cpuI_MByteEn.write(0xf);
			sig_cpuI_MCmd.write(OCP_CMD_READ);
//			sig_cpuI_MCmd.write(OCP_CMD_WRITE);
			ia += 4;
			wait(clk.posedge_event() | sig_cpuI_SCmdAccept.value_changed_event());
			if(!sig_cpuI_SCmdAccept.read())
				wait(clk.posedge_event() & sig_cpuI_SCmdAccept.value_changed_event());
			sig_cpuI_MCmd.write(OCP_CMD_IDLE);
			wait(clk.posedge_event() & sig_cpuI_SResp.value_changed_event());
			wait();

			sig_cpuD_MAddr.write(da);
			sig_cpuD_MByteEn.write(0xf);
			sig_cpuD_MCmd.write(OCP_CMD_READ);
//			sig_cpuD_MCmd.write(OCP_CMD_WRITE);
			da += 4;
			wait(clk.posedge_event() | sig_cpuD_SCmdAccept.value_changed_event());
			if(!sig_cpuD_SCmdAccept.read())
				wait(clk.posedge_event() & sig_cpuD_SCmdAccept.value_changed_event());
			sig_cpuD_MCmd.write(OCP_CMD_IDLE);
			wait(clk.posedge_event() & sig_cpuD_SResp.value_changed_event());
			wait();

			sig_cpuD_MAddr.write(0x80000000);
			sig_cpuD_MData.write('H');
			sig_cpuD_MByteEn.write(0xf);
//			sig_cpuD_MCmd.write(OCP_CMD_READ);
			sig_cpuD_MCmd.write(OCP_CMD_WRITE);
			wait(clk.posedge_event() | sig_cpuD_SCmdAccept.value_changed_event());
			if(!sig_cpuD_SCmdAccept.read())
				wait(clk.posedge_event() & sig_cpuD_SCmdAccept.value_changed_event());
			sig_cpuD_MCmd.write(OCP_CMD_IDLE);
			wait(clk.posedge_event() & sig_cpuD_SResp.value_changed_event());
			wait();

		}
	}
#endif

private:
	// Port 0 signals (fabric)
	sc_signal<sc_uint<32> > sig_p0_MAddr;
	sc_signal<sc_uint<3> >  sig_p0_MCmd;
	sc_signal<sc_uint<32> > sig_p0_MData;
	sc_signal<sc_uint<4> >  sig_p0_MByteEn;
	sc_signal<bool>         sig_p0_SCmdAccept;
	sc_signal<sc_uint<32> > sig_p0_SData;
	sc_signal<sc_uint<2> >  sig_p0_SResp;

	// Port 1 signals (fabric)
	sc_signal<sc_uint<32> > sig_p1_MAddr;
	sc_signal<sc_uint<3> >  sig_p1_MCmd;
	sc_signal<sc_uint<32> > sig_p1_MData;
	sc_signal<sc_uint<4> >  sig_p1_MByteEn;
	sc_signal<bool>         sig_p1_SCmdAccept;
	sc_signal<sc_uint<32> > sig_p1_SData;
	sc_signal<sc_uint<2> >  sig_p1_SResp;

	// CPU instruction port signals (fabric)
	sc_signal<sc_uint<32> >  sig_cpuI_MAddr;
	sc_signal<sc_uint<3> >   sig_cpuI_MCmd;
	sc_signal<sc_uint<32> >  sig_cpuI_MData;
	sc_signal<sc_uint<4> >   sig_cpuI_MByteEn;
	sc_signal<bool>          sig_cpuI_SCmdAccept;
	sc_signal<sc_uint<32> >  sig_cpuI_SData;
	sc_signal<sc_uint<2> >   sig_cpuI_SResp;

	// CPU data port signals (fabric)
	sc_signal<sc_uint<32> >  sig_cpuD_MAddr;
	sc_signal<sc_uint<3> >   sig_cpuD_MCmd;
	sc_signal<sc_uint<32> >  sig_cpuD_MData;
	sc_signal<sc_uint<4> >   sig_cpuD_MByteEn;
	sc_signal<bool>          sig_cpuD_SCmdAccept;
	sc_signal<sc_uint<32> >  sig_cpuD_SData;
	sc_signal<sc_uint<2> >   sig_cpuD_SResp;

	// CPU instruction bus signals
	sc_signal<sc_uint<32> >  sig_cpu_ib_addr;
	sc_signal<bool>          sig_cpu_ib_rdc;
	sc_signal<sc_uint<32> >  sig_cpu_ib_data;
	sc_signal<bool>          sig_cpu_ib_rdy;
	sc_signal<bool>          sig_cpu_ib_err;

	// CPU data bus signals
	sc_signal<sc_uint<32> >  sig_cpu_db_addr;
	sc_signal<bool>          sig_cpu_db_cmd;
	sc_signal<bool>          sig_cpu_db_rnw;
	sc_signal<sc_uint<4> >   sig_cpu_db_ben;
	sc_signal<sc_uint<32> >  sig_cpu_db_mdata;
	sc_signal<sc_uint<32> >  sig_cpu_db_sdata;
	sc_signal<bool>          sig_cpu_db_rdy;
	sc_signal<bool>          sig_cpu_db_err;
};
