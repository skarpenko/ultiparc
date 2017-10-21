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
 * Memory model
 */

#include <systemc.h>
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <string>
#include "ocp_defs.hxx"
#pragma once


// Memory
SC_MODULE(memory) {
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

	
	SC_HAS_PROCESS(memory);


	memory(sc_module_name mod_name, unsigned mem_size = 1*1024*1024)
		: sc_module(mod_name),
		clk("clk"), nrst("nrst"), i_MAddr("i_MAddr"), i_MCmd("i_MCmd"),
		i_MData("i_MData"), i_MByteEn("i_MByteEn"), o_SCmdAccept("o_SCmdAccept"),
		o_SData("o_SData"), o_SResp("o_SResp"),
		m_size(mem_size)
	{
		SC_THREAD(ram_proc);
			sensitive << clk.pos() << i_MCmd;
		o_SCmdAccept.initialize(true);
		// allocate memory
		m_ram = new unsigned char[m_size + 3]; // +3 for simplicity with ByteEn
	}


	~memory()
	{
		delete[] m_ram;
	}

	// Load memory from file
	void load_memory(const std::string& fname)
	{
		std::ifstream file(fname.c_str(), std::ifstream::binary);
		if(!file) {
			std::cerr << "WARNING: Failed to open file '"
				<< fname << "'!" << std::endl;
			return;
		}
		file.seekg(0, std::ios_base::end);
		std::streampos fsize = file.tellg();
		file.seekg(0, std::ios_base::beg);
		if(fsize > m_size) {
			std::cerr << "WARNING: File size is larger than memory size."
				<< " Truncated to " << m_size << " bytes." << std::endl;
			fsize = m_size;
		}
		file.read(reinterpret_cast<std::ifstream::char_type*>(m_ram), fsize);
	}

private:
	// main thread
	void ram_proc(void)
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

			bool addr_err = false;

			// Check address range
			if(addr > (m_size - 4) && addr < m_size) {
				uint32_t n = m_size - addr;
				if(ben & ~((1<<n)-1))
					addr_err = true;
			} else if(addr >= m_size) {
				addr_err = true;
			}

			// Only read and write OCP commands supported
			if(addr_err || (cmd != OCP_CMD_READ && cmd != OCP_CMD_WRITE)) {
				o_SResp = OCP_RESP_ERR;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
				continue;
			}

			if(cmd == OCP_CMD_READ) {
				// Read from RAM
				wait(clk.posedge_event());
				o_SData = *reinterpret_cast<uint32_t*>(&m_ram[addr]);
				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			} else if(cmd == OCP_CMD_WRITE) {
				// Write data to RAM according to byte enables mask
				uint32_t d = *reinterpret_cast<uint32_t*>(&m_ram[addr]);
				uint32_t mask = 0;
				if(ben&0x1) mask |= 0x000000ff;
				if(ben&0x2) mask |= 0x0000ff00;
				if(ben&0x4) mask |= 0x00ff0000;
				if(ben&0x8) mask |= 0xff000000;
				data &= mask;
				d &= ~mask;
				d |= data;
				*reinterpret_cast<uint32_t*>(&m_ram[addr]) = d;
				o_SResp = OCP_RESP_DVA;
				wait(clk.posedge_event());
				o_SResp = OCP_RESP_NULL;
			}
		}
	}

private:
	unsigned m_size;
	unsigned char *m_ram;
};
