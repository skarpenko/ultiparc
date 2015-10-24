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
 * CPU top module
 */

#include <systemc.h>
#include <stdint.h>
#include <iostream>
#include <string>
#pragma once


// CPU top
SC_MODULE(cpu_top) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	// I-Bus port
	sc_out<sc_uint<32> > ib_addr_o;
	sc_out<bool>         ib_rdc_o;
	sc_in<sc_uint<32> >  ib_data_i;
	sc_in<bool>          ib_rdy_i;
	sc_in<bool>          ib_err_i;

	// D-Bus port
	sc_out<sc_uint<32> > db_addr_o;
	sc_out<bool>         db_cmd_o;
	sc_out<bool>         db_rnw_o;
	sc_out<sc_uint<4> >  db_ben_o;
	sc_out<sc_uint<32> > db_data_o;
	sc_in<sc_uint<32> >  db_data_i;
	sc_in<bool>          db_rdy_i;
	sc_in<bool>          db_err_i;


	SC_CTOR(cpu_top) {
		SC_THREAD(cpu_test);
			sensitive << clk.pos();
	}

private:
	// Test stimuli thread
	void cpu_test(void)
	{
		unsigned ia = 0x1000;
		unsigned da = 0x2000;

		wait(nrst.posedge_event());

		while(true) {
			wait();
			ib_addr_o.write(ia);
			ib_rdc_o.write(true);
			ia += 4;
			do {
				wait(clk.posedge_event() | ib_rdy_i.value_changed_event() | ib_err_i.value_changed_event());
			} while(ib_rdy_i.read() != true && ib_err_i.read() != true);
			ib_rdc_o.write(false);
			wait();

			db_addr_o.write(da);
			db_ben_o.write(0xf);
			db_rnw_o.write(true);
			db_cmd_o.write(true);
			da += 4;
			do {
				wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
			} while(db_rdy_i.read() != true && db_err_i.read() != true);
			db_cmd_o.write(false);
			wait();

			db_addr_o.write(0x80000000);
			db_data_o.write('H');
			db_ben_o.write(0xf);
			db_rnw_o.write(false);
			db_cmd_o.write(true);
			do {
				wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
			} while(db_rdy_i.read() != true && db_err_i.read() != true);
			db_cmd_o.write(false);
			wait();

			if(ia>0x1040) {
				unsigned d = 256;
				if(ia>0x1048) d|=1;
				db_addr_o.write(0x80100000);
				db_data_o.write(d);
				db_ben_o.write(0xf);
				db_rnw_o.write(false);
				db_cmd_o.write(true);
				do {
					wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
				} while(db_rdy_i.read() != true && db_err_i.read() != true);
				db_cmd_o.write(false);
				wait();
			}
		}
	}
};
