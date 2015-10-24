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
 * CPU instruction bus adapter
 */

#include <systemc.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include "ocp_defs.hxx"
#pragma once


// I-Bus adapter
SC_MODULE(ibus_adapt) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	// CPU port
	sc_in<sc_uint<32> >   addr_i;
	sc_in<bool>           rdc_i;
	sc_out<sc_uint<32> >  data_o;
	sc_out<bool>          rdy_o;
	sc_out<bool>          err_o;

        // Fabric port
	sc_out<sc_uint<32> >  o_MAddr;
	sc_out<sc_uint<3> >   o_MCmd;
	sc_out<sc_uint<32> >  o_MData;
	sc_out<sc_uint<4> >   o_MByteEn;
	sc_in<bool>           i_SCmdAccept;
	sc_in<sc_uint<32> >   i_SData;
	sc_in<sc_uint<2> >    i_SResp;

	
	SC_CTOR(ibus_adapt) {
		SC_THREAD(ibus_proc);
			sensitive << clk.pos() << rdc_i;
	}

private:
	// main thread
	void ibus_proc(void)
	{
		wait(nrst.posedge_event());

		while(true) {
			wait();

//			rdy_o.write(false);
//			err_o.write(false);

			if(rdc_i.read() != true)
				continue;

			uint32_t addr = addr_i.read();

			// Initiate transaction to fabric
			o_MAddr.write(addr);
			o_MByteEn.write(0xF);
			o_MCmd.write(OCP_CMD_READ);

			// Wait for response
			wait(
				i_SResp.value_changed_event() |
				i_SCmdAccept.value_changed_event() |
				clk.posedge_event()
			);

			if(i_SCmdAccept == false) {
				wait(i_SCmdAccept.value_changed_event());
			}

			o_MCmd.write(OCP_CMD_IDLE);

			if(i_SResp.read() == OCP_RESP_NULL) {
				wait(i_SResp.value_changed_event());
			}

			// Send response to CPU
			if(i_SResp.read() == OCP_RESP_DVA) {
				data_o.write( i_SData.read() );
				rdy_o.write(true);
			} else {
				err_o.write(true);
			}
			wait(clk.posedge_event());
			rdy_o.write(false);
			err_o.write(false);
		}
	}
};
