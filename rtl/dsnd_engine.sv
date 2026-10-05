//============================================================================
//
//  Discrete sound engine: one small fixed-point processor that runs a per-game circuit program (MRA index 5,
//  tools/dsnd.py) once per 48 kHz sample, so each Midway one-off sound board costs a program, not logic.
//  Copyright (C) 2026 Rodimus
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

// Instruction (64 bits, little endian in the MRA; up to 1024): [63:56] op, [55:48] a, [47:40] b, [31:0] imm. Values are Q24
// (1.0 = 1 << 24), the accumulator and the 256-word state memory M are 32 bits and wrap. 2 clocks per instruction,
// at most 410 instructions per sample (824 of the 832 clocks); the sample ends at END. Output = mix >> 9, saturated (1.0 = full scale). Bit-exact with verilator/snd/dsnd_model.h.
//   0 END              1 LD a: acc = M[a]          2 LDI: acc = imm            3 ADD a: acc += M[a]
//   4 SUB a: acc -= M[a]  5 ADDI: acc += imm       6 MULI: acc = acc * imm >> 24   7 MUL a: acc = acc * M[a] >> 24
//   8 ST a: M[a] = acc  9 STF a: if F           10 LDIF: if F acc = imm
//  11 RC a: M[a] += (acc - M[a]) * imm >> 24, acc = M[a]   12 RCM a, b: the same with the coefficient M[b]
//  13 CMP a: F = acc >= M[a]   14 CMPI: F = acc >= imm   15 MAXI   16 MINI
//  17 BIT b: F = source bit (a[0] inverts)   18 NOT: F = !F   19 NOISE b: step LFSR b[1:0], F = its new bit
//  20 OUT: mix += acc   21 LDF a: if F acc = M[a]   22 SKF: if F skip imm   23 SKNF: if !F skip imm
//  24 LDL b: acc = source byte << 16   25 ABS   26 FAND b: F &= bit   27 FOR b: F |= bit   28 STNF a: if !F
//  29 LDLM b: acc = (source byte & imm[7:0]) << imm[12:8]   30 MACI a: acc += M[a] * imm >> 24
// Sources (b[5:3]): 0-3 sound latches 1-4, 4 latch 0, 5 misc, 6 / 7 Game Audio settings; b[2:0] = bit.

module dsnd_engine
(
    input  logic        clk,            // 39.936 MHz
    input  logic        reset,
    input  logic        pause,
    input  logic        prog_wr,        // MRA index 5 byte
    input  logic [12:0] prog_addr,
    input  logic  [7:0] prog_data,
    input  logic [63:0] src,            // sources 7..0, one byte each
    output logic signed [15:0] out
);

// Two clocks per instruction: E1 reads M[a] / M[b] (addressed from the program word in the previous E2), registers
// the multiplier operands and starts the next program fetch; E2 multiplies, writes back and addresses the next M
// reads. A write in E2 lands on the same edge as the next instruction's M read, so that read is forwarded.
typedef enum logic [2:0] { S_IDLE, S_LOAD, S_E1, S_E2, S_PUB, S_CLR } state_t;
state_t st = S_CLR;

logic  [9:0] pc = 10'd0, pa;
logic [63:0] pq;
dpram_dc #(.widthad_a(10), .width_a(64)) u_prog
(
    .clock_a(clk),
    .address_a(prog_wr ? prog_addr[12:3] : pa),
    .data_a({8{prog_data}}),
    .wren_a(prog_wr),
    .byteena_a(8'd1 << prog_addr[2:0]),
    .q_a(pq),

    .clock_b(clk),
    .address_b(10'd0),
    .data_b(64'd0),
    .wren_b(1'b0),
    .byteena_b(8'hFF),
    .q_b()
);

// state memory, two copies (read M[a] / M[b]) sharing the write port
logic  [7:0] wa;
logic [31:0] wd;
logic        we;
logic [31:0] mqa, mqb;
wire   [7:0] ra = pq[55:48], rb = pq[47:40];
dpram_dc #(.widthad_a(8), .width_a(32)) u_ma
(
    .clock_a(clk), .address_a(ra), .data_a(32'd0), .wren_a(1'b0), .byteena_a(4'hF), .q_a(mqa),
    .clock_b(clk), .address_b(wa), .data_b(wd),    .wren_b(we),    .byteena_b(4'hF), .q_b()
);
dpram_dc #(.widthad_a(8), .width_a(32)) u_mb
(
    .clock_a(clk), .address_a(rb), .data_a(32'd0), .wren_a(1'b0), .byteena_a(4'hF), .q_a(mqb),
    .clock_b(clk), .address_b(wa), .data_b(wd),    .wren_b(we),    .byteena_b(4'hF), .q_b()
);

// sample timing
logic [9:0] tick = 10'd0;
wire        go = tick == 10'd831;
always_ff @(posedge clk) if (reset) tick <= 10'd0; else if (!pause) tick <= go ? 10'd0 : tick + 10'd1;

logic [63:0] ir = 64'd0;
logic signed [31:0] acc = 32'sd0, mix = 32'sd0;
logic        f = 1'b0;
logic [30:0] lfsr[4] = '{31'd1, 31'd1, 31'd1, 31'd1};
logic  [8:0] icnt = 9'd0;
logic signed [32:0] mul_a = 33'sd0;
logic signed [31:0] mul_b = 32'sd0;
logic signed [31:0] ma_r = 32'sd0;              // M[a] as E1 saw it
logic        fwd_a = 1'b0, fwd_b = 1'b0;
logic [31:0] fwd_d = 32'd0;
logic  [7:0] clr = 8'd0;

wire  [7:0] op  = ir[63:56];
wire  [7:0] fa  = ir[55:48];
wire  [7:0] fb  = ir[47:40];
wire signed [31:0] imm = ir[31:0];
wire signed [31:0] m_a = fwd_a ? fwd_d : mqa;   // E1: M[a] / M[b] with the previous write forwarded
wire signed [31:0] m_b = fwd_b ? fwd_d : mqb;
wire  [2:0] ssel = fb[5:3];
wire  [7:0] sbyte = src[ssel*8 +: 8];
wire        sbit = sbyte[fb[2:0]] ^ fa[0];
wire signed [64:0] prod  = mul_a * mul_b;
wire signed [31:0] mul_q = 32'(prod >>> 24);

// E2 results
logic signed [31:0] acc_n, wd_n;
logic        f_n, we_n;
always_comb begin
    acc_n = acc; f_n = f; we_n = 1'b0; wd_n = acc;
    case (op)
        8'd1:  acc_n = ma_r;
        8'd2:  acc_n = imm;
        8'd3:  acc_n = acc + ma_r;
        8'd4:  acc_n = acc - ma_r;
        8'd5:  acc_n = acc + imm;
        8'd6, 8'd7: acc_n = mul_q;
        8'd30: acc_n = acc + mul_q;
        8'd8:  we_n = 1'b1;
        8'd9:  we_n = f;
        8'd28: we_n = ~f;
        8'd10: if (f) acc_n = imm;
        8'd11, 8'd12: begin acc_n = ma_r + mul_q; wd_n = ma_r + mul_q; we_n = 1'b1; end
        8'd13: f_n = acc >= ma_r;
        8'd14: f_n = acc >= imm;
        8'd15: if (acc < imm) acc_n = imm;
        8'd16: if (acc > imm) acc_n = imm;
        8'd17: f_n = sbit;
        8'd18: f_n = ~f;
        8'd21: if (f) acc_n = ma_r;
        8'd24: acc_n = {8'd0, sbyte, 16'd0};
        8'd29: acc_n = {24'd0, sbyte & imm[7:0]} << imm[12:8];
        8'd25: if (acc < 0) acc_n = -acc;
        8'd26: f_n = f & sbit;
        8'd27: f_n = f | sbit;
        default: ;
    endcase
end

// program address: the next instruction is fetched in E1 (skips decided by F, which is settled by then)
wire [9:0] pc_next = pc + 10'd1 + (((op == 8'd22 && f) || (op == 8'd23 && !f)) ? 10'(imm) : 10'd0);
assign pa = (st == S_E1) ? pc_next : pc;

// writes: E2 results, or clearing after reset
assign wa = (st == S_CLR) ? clr : fa;
assign wd = (st == S_CLR) ? 32'd0 : wd_n;
assign we = (st == S_CLR) | (st == S_E2 & we_n);

always_ff @(posedge clk) begin
    if (reset) begin
        st  <= S_CLR;
        clr <= 8'd0;
        out <= 16'sd0;
    end else case (st)
        S_CLR: begin                                    // clear the state memory after reset
            clr <= clr + 8'd1;
            if (clr == 8'd255) st <= S_IDLE;
        end
        S_IDLE: if (go) begin
            pc   <= 10'd0;
            mix  <= 32'sd0;
            icnt <= 9'd0;
            st   <= S_LOAD;
        end
        S_LOAD: begin                                   // program word 0 being read (pa = pc = 0)
            ir    <= 64'd0;
            fwd_a <= 1'b0;
            fwd_b <= 1'b0;
            st    <= S_E2;                              // a NOP E2 addresses the first instruction's M reads
            icnt  <= 9'd0;
        end
        S_E1: begin
            ma_r <= m_a;
            case (op)
                8'd6:  begin mul_a <= 33'(acc); mul_b <= imm; end
                8'd7:  begin mul_a <= 33'(acc); mul_b <= m_a; end
                8'd30: begin mul_a <= 33'(m_a); mul_b <= imm; end
                8'd11: begin mul_a <= 33'(acc) - 33'(m_a); mul_b <= imm; end
                8'd12: begin mul_a <= 33'(acc) - 33'(m_a); mul_b <= m_b; end
                default: ;
            endcase
            pc <= pc_next;
            st <= S_E2;
        end
        S_E2: begin
            acc <= acc_n;
            f   <= f_n;
            if (op == 8'd19) begin : noise
                logic nb;
                nb = lfsr[fb[1:0]][28] ^ lfsr[fb[1:0]][0];      // MAME SN76477 / discrete 31-bit noise taps
                lfsr[fb[1:0]] <= {nb, lfsr[fb[1:0]][30:1]};
                f <= nb;
            end
            if (op == 8'd20) mix <= mix + acc;
            // the next instruction: its word is on pq now; its M reads are addressed this edge
            ir    <= pq;
            fwd_a <= we_n && pq[55:48] == fa && icnt != 9'd0;
            fwd_b <= we_n && pq[47:40] == fa && icnt != 9'd0;
            fwd_d <= wd_n;
            icnt  <= icnt + 9'd1;
            st    <= S_E1;
            if (icnt != 9'd0 && (op == 8'd0 || icnt == 9'd410)) st <= S_PUB;   // END, or the 410-instruction budget
        end
        S_PUB: begin                                    // publish the sample (1.0 = full scale)
            out <= (mix >>> 9) > 32'sd32767 ? 16'sd32767 : (mix >>> 9) < -32'sd32768 ? -16'sd32768 : 16'(mix >>> 9);
            st  <= S_IDLE;
        end
        default: st <= S_IDLE;
    endcase
end

endmodule
