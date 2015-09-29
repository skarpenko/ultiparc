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
 * Simulation control device
 */

#include <systemc.h>
#include <iostream>
#include <string>
#include "ocp_defs.hxx"
#pragma once


// Simulation control
//  Registers:
//    0x00  -  control register;
//             Bits:
//              [31:1] - ignored;
//              [0]    - stop simulation.
SC_MODULE(sim_ctrl) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	// Slave port
	sc_in<sc_uint<32> >  i_MAddr;
	sc_in<sc_uint<3> >   i_MCmd;
	sc_in<sc_uint<32> >  i_MData;
	sc_in<sc_uint<4> >   i_MByteEn;
	sc_out<bool>         o_SCmdAccept;
	sc_out<sc_uint<32> > o_SData;
	sc_out<sc_uint<2> >  o_SResp;


	SC_CTOR(sim_ctrl) {
		SC_THREAD(sim_ctrl_proc);
			sensitive << clk.pos() << i_MCmd;
		o_SCmdAccept.initialize(true);
		last_value = 0;
	}

public:
	unsigned int last_value;  // last value written to control register

private:
	// main thread
	void sim_ctrl_proc(void)
	{
		wait(nrst.posedge_event());

		while(true) {
			wait();

			unsigned addr = i_MAddr.read();
			unsigned data = i_MData.read();
			unsigned cmd  = i_MCmd.read();
			unsigned ben  = i_MByteEn.read();

			if(cmd == OCP_CMD_IDLE)
				continue;

			// Address must be 0
			bool addr_err = (addr != 0);

			// Only read and write OCP commands supported
			if(addr_err || (cmd != OCP_CMD_READ && cmd != OCP_CMD_WRITE)) {
				o_SResp = OCP_RESP_ERR;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
				continue;
			}

			if(cmd == OCP_CMD_READ) {
	                        // Return last written value
				wait(clk.posedge_event());
				o_SData = last_value;
				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			} else if(cmd == OCP_CMD_WRITE) {
				// Store value
				unsigned mask = 0;
				if(ben&0x1) mask |= 0x000000ff;
				if(ben&0x2) mask |= 0x0000ff00;
				if(ben&0x4) mask |= 0x00ff0000;
				if(ben&0x8) mask |= 0xff000000;
				data &= mask;
				last_value &= ~mask;
				last_value |= data;

				// If simulation termination requested
				if(last_value & 0x1) {
					std::cout << std::endl;
					sc_stop();
				}

				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			}
		}
	}
};
