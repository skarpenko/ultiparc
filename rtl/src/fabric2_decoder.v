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
 * System fabric (version 2)
 */

`include "common.vh"
`include "ocp_const.vh"


/* Address decoder */
module fabric2_decoder #(
	parameter PORTNO_WIDTH = 11
)
(
	i_addr,
	o_addr,
	o_portno
);
localparam PORT_BITS = PORTNO_WIDTH + 1;

input wire [`ADDR_WIDTH-1:0]		i_addr;
output wire [`ADDR_WIDTH-1:0]		o_addr;
output wire [PORTNO_WIDTH-1:0]		o_portno;


/* Decode address */
assign o_addr = (!i_addr[`ADDR_WIDTH-1] ? { 1'b0, i_addr[`ADDR_WIDTH-2:0] } :
		{ {(PORT_BITS){1'b0}}, i_addr[`ADDR_WIDTH-PORT_BITS-1:0] });


/* Decode port number */
assign o_portno = (i_addr[`ADDR_WIDTH-1] ? i_addr[`ADDR_WIDTH-2:`ADDR_WIDTH-PORT_BITS] + 1'b1 :
		{(PORT_BITS-1){1'b0}});


endmodule /* fabric2_decoder */
