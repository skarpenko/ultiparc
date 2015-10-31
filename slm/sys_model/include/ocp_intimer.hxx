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
 * Interval Timer module
 */

#include <systemc.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include "ocp_defs.hxx"
#pragma once


// Interval Timer
//  Registers:
//    0x00  -  control register;
//             Bits:
//              [31:3] - ignored;
//              [2]    - reload value (1);
//              [1]    - mask (0) or unmask (1) interrupt;
//              [0]    - enable (1) and disable (0) timer.
//    0x04  -  counter register;
//    0x08  -  current count register (read only).
SC_MODULE(intimer) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	sc_out<bool>         intr_o; // interrupt signal

	// Slave port
	sc_in<sc_uint<32> >  i_MAddr;
	sc_in<sc_uint<3> >   i_MCmd;
	sc_in<sc_uint<32> >  i_MData;
	sc_in<sc_uint<4> >   i_MByteEn;
	sc_out<bool>         o_SCmdAccept;
	sc_out<sc_uint<32> > o_SData;
	sc_out<sc_uint<2> >  o_SResp;


	SC_CTOR(intimer) {
		SC_CTHREAD(intimer_count_proc, clk.pos());
		SC_THREAD(intimer_ctrl_proc);
			sensitive << clk.pos() << i_MCmd;
		o_SCmdAccept.initialize(true);
		m_ctrl = 0;
		m_counter = 0;
		m_current = 0;
	}

public:
	uint32_t m_ctrl;	// Control register value
	uint32_t m_counter;	// Initial counter value
	uint32_t m_current;	// Current count

private:
	// counting thread
	void intimer_count_proc(void)
	{
		while(true) {
			wait();
			if(!nrst.read())
				continue;

			if(intr_o.read())
				intr_o.write(false);

			if(!(m_ctrl & 0x01))
				continue;

			if(!m_current && (m_ctrl & 0x04)) {
				m_current = m_counter;
				continue;
			} else if(!m_current && !(m_ctrl & 0x04)) {
				continue;
			}

			if(!(--m_current))
				if(m_ctrl & 0x02) {
					intr_o.write(true);
					wait();
				}
		}
	}

	// main control thread
	void intimer_ctrl_proc(void)
	{
		wait(nrst.posedge_event());

		while(true) {
			wait();

			uint32_t addr = i_MAddr.read();
			uint32_t data = i_MData.read();
			uint32_t cmd  = i_MCmd.read();
			uint32_t ben  = i_MByteEn.read();

			if(cmd == OCP_CMD_IDLE)
				continue;

			// Address must be 0x00, 0x04 or 0x08
			bool addr_err = ((addr != 0x00) && (addr != 0x04) && (addr != 0x08));

			// Only read and write OCP commands supported
			if(addr_err || (cmd != OCP_CMD_READ && cmd != OCP_CMD_WRITE)) {
				o_SResp = OCP_RESP_ERR;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
				continue;
			}

			if(cmd == OCP_CMD_READ) {
				wait(clk.posedge_event());
				uint32_t val;
				if(addr == 0x00)
					val = m_ctrl;
				else if(addr == 0x04)
					val = m_counter;
				else
					val = m_current;
				o_SData = val;
				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			} else if(cmd == OCP_CMD_WRITE) {
				// Store value
				uint32_t mask = 0;
				if(ben&0x1) mask |= 0x000000ff;
				if(ben&0x2) mask |= 0x0000ff00;
				if(ben&0x4) mask |= 0x00ff0000;
				if(ben&0x8) mask |= 0xff000000;
				data &= mask;

				if(addr == 0x00) {
					m_ctrl &= ~mask;
					m_ctrl |= data;
				} else if(addr == 0x04) {
					m_counter &= ~mask;
					m_counter |= data;
					m_current = m_counter; // reload
				}

				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			}
		}
	}
};
