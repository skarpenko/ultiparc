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
 * Interrupt controller module
 */

#include <systemc.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include "ocp_defs.hxx"
#pragma once


// Interrupt controller
//  Registers:
//    0x00  -  interrupt status register;
//             Bits:
//              [31:0] - set bits denotes active unmasked interrupt lines.
//             Write to this register acknowledges specified interrupts.
//    0x04  -  interrupt mask register;
//             Bits:
//              [31:0] - set bits correspond to unmasked interrupts.
//    0x08  -  raw interrupts register (read only register);
//             Bits:
//              [31:0] - set bits correspond to active interrupt lines.
SC_MODULE(intctrl) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	sc_in<bool>          intr0_i;

	sc_out<bool>         cpu_intr_o; // interrupt signal to CPU


	// Slave port
	sc_in<sc_uint<32> >  i_MAddr;
	sc_in<sc_uint<3> >   i_MCmd;
	sc_in<sc_uint<32> >  i_MData;
	sc_in<sc_uint<4> >   i_MByteEn;
	sc_out<bool>         o_SCmdAccept;
	sc_out<sc_uint<32> > o_SData;
	sc_out<sc_uint<2> >  o_SResp;


	SC_CTOR(intctrl) {
		SC_METHOD(intctrl_intr_detect_meth);
			sensitive << intr0_i;
		SC_CTHREAD(intctrl_intr_delivery_proc, clk.pos());
		SC_THREAD(intctrl_ctrl_proc);
			sensitive << clk.pos() << i_MCmd;
		o_SCmdAccept.initialize(true);
		m_mask = 0;
		m_raw = 0;
	}

public:
	uint32_t m_mask;	// Interrupts mask
	uint32_t m_raw;		// Current active interrupt lines

private:
	// Read interrupt status register
	uint32_t rd_status_reg(void)
	{
		return m_raw & m_mask;
	}

	// Write interrupt status register (Ack interrupts)
	void wr_status_reg(uint32_t v)
	{
		m_raw &= ~v;
	}
	

	// Interrupt detection
	void intctrl_intr_detect_meth(void)
	{
		if(intr0_i.read())
			m_raw |= 0x01;
	}

	// Interrupt delivery thread
	void intctrl_intr_delivery_proc(void)
	{
		while(true) {
			wait();
			if(!nrst.read())
				continue;

			if(rd_status_reg())
				cpu_intr_o.write(true);
			else
				cpu_intr_o.write(false);
		}
	}

	// main control thread
	void intctrl_ctrl_proc(void)
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
					val = rd_status_reg();
				else if(addr == 0x04)
					val = m_mask;
				else
					val = m_raw;
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
					uint32_t val = rd_status_reg();
					val &= ~mask;
					val |= data;
					wr_status_reg(val);
				} else if(addr == 0x04) {
					m_mask &= ~mask;
					m_mask |= data;
				}

				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			}
		}
	}
};
