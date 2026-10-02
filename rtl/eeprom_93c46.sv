//============================================================================
//
//  93C46 serial EEPROM, 8-bit organisation (128 x 8)
//  Copyright (C) 2026 Rodimus
//
//  Behaviour follows MAME eepromser.cpp / eeprom.cpp (eeprom_serial_93cxx_device):
//  start bit, 2 opcode + 7 address bits, READ = dummy 0 then 8 data bits, WRITE / ERASE / ERAL / WRAL refused
//  while locked (EWDS state at reset), WRAL ANDs into the cells, busy 1.75 ms write / 1 ms erase / 8 ms all.
//  DO reads READY while waiting for a start bit, the data bit while reading, otherwise 1 (pull-up).
//
//  Redistribution and use in source and synthesized forms, with or without
//  modification, are permitted provided that the following conditions are met:
//  Redistributions of source code must retain the above copyright notice,
//  this list of conditions and the following disclaimer. Redistributions in
//  synthesized form must reproduce the above copyright notice, this list of
//  conditions and the following disclaimer in the documentation and/or other
//  materials provided with the distribution. Neither the name of the author
//  nor the names of other contributors may be used to endorse or promote
//  products derived from this software without specific prior written
//  permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
//  AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY OUT OF
//  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//============================================================================

// clk = 39.936 MHz. Cells are stored inverted so the power-up (all 0) RAM reads as erased (FF).

module eeprom_93c46
(
    input  logic       clk,
    input  logic       reset,        // device reset: lock, idle (contents kept)
    input  logic       clear,        // erase every cell (new ROM download)

    input  logic       wr,           // one-clock strobe: the CPU wrote the control byte
    input  logic       di,
    input  logic       sk,
    input  logic       cs,

    output logic       dout
);

localparam logic [2:0] S_RESET = 3'd0, S_START = 3'd1, S_CMD = 3'd2, S_READ = 3'd3, S_DATA = 3'd4, S_DONE = 3'd5;

localparam int T_WRITE = 69888;      // 1750 us
localparam int T_ERASE = 39936;      // 1000 us
localparam int T_ALL   = 319488;     // 8000 us

logic  [2:0] state = S_RESET;
logic        cs_q = 1'b0, sk_q = 1'b0, di_q = 1'b0;
logic        locked = 1'b1;
logic  [8:0] cmd = 9'd0;             // opcode [8:7], address [6:0]
logic  [3:0] nbits = 4'd0;
logic  [6:0] addr = 7'd0;
logic  [7:0] shift = 8'd0;           // read: [7] = DO; write: data in
logic  [7:0] wdata = 8'd0;
logic [18:0] busy = 19'd0;

// whole-array sweep (clear, ERAL, WRAL): read a cell, then write it
logic        sweep = 1'b0, sw_ph = 1'b0, sw_and = 1'b0;
logic  [6:0] sw_a = 7'd0;

logic  [7:0] q_n;                    // stored (inverted) cell at mem_ra
logic  [6:0] mem_wa;
logic  [7:0] mem_wd;
logic        mem_we;
wire   [6:0] mem_ra = sweep ? sw_a : addr;
wire   [7:0] q = ~q_n;

dpram_dc #(.widthad_a(7)) u_mem
(
    .clock_a(clk),
    .address_a(mem_wa),
    .data_a(mem_wd),
    .wren_a(mem_we),
    .q_a(),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b(mem_ra),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(q_n),
    .byteena_b(1'b1)
);

wire ready = (busy == 19'd0) && !sweep;

always_comb begin
    case (state)
        S_START: dout = ready;
        S_READ:  dout = shift[7];
        default: dout = 1'b1;
    endcase
end

always_ff @(posedge clk) begin
    mem_we <= 1'b0;
    if (busy != 19'd0) busy <= busy - 19'd1;

    // array sweep: phase 0 sets the read address, phase 1 writes the cell
    if (sweep) begin
        sw_ph <= ~sw_ph;
        if (sw_ph) begin
            mem_wa <= sw_a;
            mem_wd <= ~(sw_and ? q & wdata : 8'hFF);
            mem_we <= 1'b1;
            sw_a   <= sw_a + 7'd1;
            if (sw_a == 7'd127) sweep <= 1'b0;
        end
    end

    if (clear) begin
        sweep  <= 1'b1;
        sw_ph  <= 1'b0;
        sw_a   <= 7'd0;
        sw_and <= 1'b0;
        state  <= S_RESET;
        locked <= 1'b1;
    end
    else if (reset) begin
        state  <= S_RESET;
        locked <= 1'b1;
        cs_q   <= 1'b0;
        sk_q   <= 1'b0;
    end
    else if (wr) begin
        cs_q <= cs;
        sk_q <= sk;
        di_q <= di;
        if (cs_q && !cs)
            state <= S_RESET;
        else if (!cs_q && cs)
            state <= S_START;                                      // a clock edge in the same write is ignored
        else if (cs && sk && !sk_q) begin
            case (state)
                S_START:
                    if (di && ready) begin
                        state <= S_CMD;
                        nbits <= 4'd0;
                        cmd   <= 9'd0;
                    end
                S_CMD: begin
                    cmd   <= {cmd[7:0], di};
                    nbits <= nbits + 4'd1;
                    if (nbits == 4'd8) begin                       // 9th bit: opcode + address complete
                        nbits <= 4'd0;
                        addr  <= {cmd[5:0], di};
                        case (cmd[7:6])
                            2'd2: begin state <= S_READ; shift <= 8'd0; end    // READ: dummy 0 first
                            2'd1: state <= S_DATA;                              // WRITE
                            2'd3: if (locked) state <= S_RESET;                 // ERASE
                                  else begin
                                      mem_wa <= {cmd[5:0], di};
                                      mem_wd <= 8'h00;
                                      mem_we <= 1'b1;
                                      busy   <= 19'(T_ERASE);
                                      state  <= S_DONE;
                                  end
                            default:
                                case (cmd[5:4])
                                    2'd0: begin locked <= 1'b1; state <= S_RESET; end            // EWDS
                                    2'd3: begin locked <= 1'b0; state <= S_RESET; end            // EWEN
                                    2'd1: state <= S_DATA;                                       // WRAL
                                    default: if (locked) state <= S_RESET;                       // ERAL
                                             else begin
                                                 sweep <= 1'b1; sw_ph <= 1'b0; sw_a <= 7'd0; sw_and <= 1'b0;
                                                 busy  <= 19'(T_ALL);
                                                 state <= S_DONE;
                                             end
                                endcase
                        endcase
                    end
                end
                S_READ: begin
                    nbits <= (nbits == 4'd15) ? nbits : nbits + 4'd1;
                    shift <= (nbits == 4'd0) ? q : {shift[6:0], 1'b1};  // first clock fetches the cell
                end
                S_DATA: begin
                    shift <= {shift[6:0], di};
                    nbits <= nbits + 4'd1;
                    if (nbits == 4'd7) begin
                        if (locked)
                            state <= S_RESET;
                        else if (cmd[8:7] == 2'd1) begin                // WRITE
                            mem_wa <= addr;
                            mem_wd <= ~{shift[6:0], di};
                            mem_we <= 1'b1;
                            busy   <= 19'(T_WRITE);
                            state  <= S_DONE;
                        end else begin                                  // WRAL
                            wdata <= {shift[6:0], di};
                            sweep <= 1'b1; sw_ph <= 1'b0; sw_a <= 7'd0; sw_and <= 1'b1;
                            busy  <= 19'(T_ALL);
                            state <= S_DONE;
                        end
                    end
                end
                default: ;
            endcase
        end
    end
end

endmodule
