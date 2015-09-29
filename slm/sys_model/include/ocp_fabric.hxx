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
 * Simple OCP fabric model
 */

#include <systemc.h>
#include "ocp_defs.hxx"
#pragma once


// Fabric
SC_MODULE(fabric) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	// Instruction slave port
	sc_in<sc_uint<32> >  i_cpuI_MAddr;
	sc_in<sc_uint<3> >   i_cpuI_MCmd;
	sc_in<sc_uint<32> >  i_cpuI_MData;
	sc_in<sc_uint<4> >   i_cpuI_MByteEn;
	sc_out<bool>         o_cpuI_SCmdAccept;
	sc_out<sc_uint<32> > o_cpuI_SData;
	sc_out<sc_uint<2> >  o_cpuI_SResp;

	// Data slave port
	sc_in<sc_uint<32> >  i_cpuD_MAddr;
	sc_in<sc_uint<3> >   i_cpuD_MCmd;
	sc_in<sc_uint<32> >  i_cpuD_MData;
	sc_in<sc_uint<4> >   i_cpuD_MByteEn;
	sc_out<bool>         o_cpuD_SCmdAccept;
	sc_out<sc_uint<32> > o_cpuD_SData;
	sc_out<sc_uint<2> >  o_cpuD_SResp;

	static const int NPORTS = 3; // Total master ports number

	// Master ports
	sc_out<sc_uint<32> > o_px_MAddr[NPORTS];
	sc_out<sc_uint<3> >  o_px_MCmd[NPORTS];
	sc_out<sc_uint<32> > o_px_MData[NPORTS];
	sc_out<sc_uint<4> >  o_px_MByteEn[NPORTS];
	sc_in<bool>          i_px_SCmdAccept[NPORTS];
	sc_in<sc_uint<32> >  i_px_SData[NPORTS];
	sc_in<sc_uint<2> >   i_px_SResp[NPORTS];

	// Per port address ranges for address decoding
	static const unsigned PX_BASE_ADDR[NPORTS];
	static const unsigned PX_LAST_ADDR[NPORTS];


	SC_CTOR(fabric) {
		SC_THREAD(fab_cpu_ports_thread);
			sensitive << clk.pos()
				  << i_cpuI_MCmd
				  << i_cpuD_MCmd;

		o_cpuI_SCmdAccept.initialize(true);
		o_cpuD_SCmdAccept.initialize(true);
	}

private:
	// main thread
	void fab_cpu_ports_thread(void)
	{
		wait(nrst.posedge_event());

		while(true) {
			wait();

			// Transaction on instruction fetch port
			if(i_cpuI_MCmd.read() != OCP_CMD_IDLE)
				fab_cpuI_proc();

			// Transaction on data fetch port
			if(i_cpuD_MCmd.read() != OCP_CMD_IDLE)
				fab_cpuD_proc();
		}
	}

	// Handle instructions fetch
	void fab_cpuI_proc(void)
	{
		unsigned cmd  = i_cpuI_MCmd.read();
		unsigned addr = i_cpuI_MAddr.read();

		if(cmd == OCP_CMD_IDLE)
			return;

		// Select destination master port
		int pi = -1;
		for(int i=0; i<NPORTS; ++i) {
			if(addr >= PX_BASE_ADDR[i] && addr <= PX_LAST_ADDR[i]) {
				pi = i;
				break;
			}
		}

		if(pi<0) {
			o_cpuI_SResp = OCP_RESP_ERR;
			wait();
			o_cpuI_SResp = OCP_RESP_NULL;
			return;
		}

		addr -= PX_BASE_ADDR[pi];

		// Initiate new transaction
		o_px_MAddr[pi] = addr;
		o_px_MData[pi] = i_cpuI_MData;
		o_px_MByteEn[pi] = i_cpuI_MByteEn;
		o_px_MCmd[pi] = i_cpuI_MCmd;

		wait(
			i_px_SResp[pi].value_changed_event() |
			i_px_SCmdAccept[pi].value_changed_event() |
			clk.posedge_event()
		);

		if(i_px_SCmdAccept[pi] == false) {
			o_cpuI_SCmdAccept = i_px_SCmdAccept[pi];
			wait(i_px_SCmdAccept[pi].value_changed_event());
			o_cpuI_SCmdAccept = i_px_SCmdAccept[pi];
		}

		// Set response to initiator
		if(i_px_SResp[pi].read() != OCP_RESP_NULL) {
			o_px_MCmd[pi] = i_cpuI_MCmd;
			o_cpuI_SData = i_px_SData[pi];
			o_cpuI_SResp = i_px_SResp[pi];
			wait(i_px_SResp[pi].value_changed_event());
			o_cpuI_SData = i_px_SData[pi];
			o_cpuI_SResp = i_px_SResp[pi];
		} else {
			wait(i_px_SResp[pi].value_changed_event());
			o_px_MCmd[pi] = i_cpuI_MCmd;
			o_cpuI_SData = i_px_SData[pi];
			o_cpuI_SResp = i_px_SResp[pi];
			wait(i_px_SResp[pi].value_changed_event());
			o_cpuI_SResp = i_px_SResp[pi];
		}
	}

	// Handle data fetch
	void fab_cpuD_proc(void)
	{
		unsigned cmd  = i_cpuD_MCmd.read();
		unsigned addr = i_cpuD_MAddr.read();

		if(cmd == OCP_CMD_IDLE)
			return;

		// Select destination master port
		int pi = -1;
		for(int i=0; i<NPORTS; ++i) {
			if(addr >= PX_BASE_ADDR[i] && addr <= PX_LAST_ADDR[i]) {
				pi = i;
				break;
			}
		}

		if(pi<0) {
			o_cpuD_SResp = OCP_RESP_ERR;
			wait();
			o_cpuD_SResp = OCP_RESP_NULL;
			return;
		}

		addr -= PX_BASE_ADDR[pi];

		// Start new transaction
		o_px_MAddr[pi] = addr;
		o_px_MData[pi] = i_cpuD_MData;
		o_px_MByteEn[pi] = i_cpuD_MByteEn;
		o_px_MCmd[pi] = i_cpuD_MCmd;

		wait(
			i_px_SResp[pi].value_changed_event() |
			i_px_SCmdAccept[pi].value_changed_event() |
			clk.posedge_event()
		);

		if(i_px_SCmdAccept[pi] == false) {
			o_cpuD_SCmdAccept = i_px_SCmdAccept[pi];
			wait(i_px_SCmdAccept[pi].value_changed_event());
			o_cpuD_SCmdAccept = i_px_SCmdAccept[pi];
		}

		// Set response to initiator
		if(i_px_SResp[pi].read() != OCP_RESP_NULL) {
			o_px_MCmd[pi] = i_cpuD_MCmd;
			o_cpuD_SData = i_px_SData[pi];
			o_cpuD_SResp = i_px_SResp[pi];
			wait(i_px_SResp[pi].value_changed_event());
			o_cpuD_SData = i_px_SData[pi];
			o_cpuD_SResp = i_px_SResp[pi];
		} else {
			wait(i_px_SResp[pi].value_changed_event());
			o_px_MCmd[pi] = i_cpuD_MCmd;
			o_cpuD_SData = i_px_SData[pi];
			o_cpuD_SResp = i_px_SResp[pi];
			wait(i_px_SResp[pi].value_changed_event());
			o_cpuD_SResp = i_px_SResp[pi];
		}
	}
};

const unsigned fabric::PX_BASE_ADDR[] = { 0x00000000, 0x80000000, 0x80100000 };
const unsigned fabric::PX_LAST_ADDR[] = { 0x0fffffff, 0x800fffff, 0x801fffff };
