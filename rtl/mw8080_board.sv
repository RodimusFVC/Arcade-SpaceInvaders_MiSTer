//============================================================================
//
//  Midway / Taito 8080 black and white board (Space Invaders hardware)
//  Copyright (C) 2026 Rodimus
//
//  Timing, RAM arbitration and interrupt logic from mw8080.vhd / invaders.vhd,
//  Copyright (c) 2002 Daniel Wallner, MikeJ tidy-up, MiSTer port by Gehstock,
//  Gyurco, David Woods, Mike Coates, Shane Lynch and Alan Steremberg.
//  Memory map, I/O map and shifter follow MAME mw8080bw.cpp / 8080bw.cpp.
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

// clk = 39.936 MHz (2 x the 19.968 MHz board crystal). ce = 9.984 MHz, pixel = ce/2 = 4.992 MHz, CPU = ce/5.
// Picture = MAME's 260 x 224: 4 black pixels, then the 256 bitmap pixels.

module mw8080_board
(
    input  logic        clk,
    input  logic        reset,
    input  logic        pause,
    input  logic  [7:0] variant,

    input  logic  [7:0] in0,
    input  logic  [7:0] in1,
    input  logic  [7:0] in2,
    input  logic        cocktail,

    input  logic [24:0] ioctl_addr,
    input  logic  [7:0] ioctl_dout,
    input  logic        ioctl_wr0,

    input  logic        crt_flip,

    output logic        ce_pix,
    output logic  [7:0] video_r,
    output logic  [7:0] video_g,
    output logic  [7:0] video_b,
    output logic        video_hs,
    output logic        video_vs,
    output logic        video_hblank,
    output logic        video_vblank,

    output logic  [7:0] snd1,
    output logic  [7:0] snd2,
    output logic signed [15:0] audio,

    input  logic [15:0] hs_address,
    input  logic  [7:0] hs_data_in,
    output logic  [7:0] hs_data_out,
    input  logic        hs_write
);

// ---------------------------------------------------------------- clock enables

logic [1:0] ce_div = 2'd0;
always_ff @(posedge clk) ce_div <= ce_div + 2'd1;
wire ce = (ce_div == 2'd3);

logic       vid_en = 1'b0;          // pixel enable, every other ce
logic [2:0] cpu_cnt = 3'd0;         // CPU enable, 1 ce in 5
always_ff @(posedge clk) if (ce) begin
    vid_en  <= ~vid_en;
    cpu_cnt <= (cpu_cnt == 3'd4) ? 3'd0 : cpu_cnt + 3'd1;
end
wire pix    = ce & vid_en;
wire cpu_ce = ce & (cpu_cnt == 3'd4);
assign ce_pix = pix;

// ---------------------------------------------------------------- reset / watchdog

logic       rst_n = 1'b0;
logic       wd_reset = 1'b0;
always_ff @(posedge clk) rst_n <= ~(reset | wd_reset);

// ---------------------------------------------------------------- video counters (D5/E5 horizontal, E6/E7 vertical)

logic [3:0] cnt_d5 = 4'd0;          // 1H..8H
logic [4:0] cnt_e5 = 5'd0;          // 16H..128H, [4] = HBLANK
logic [3:0] cnt_e6 = 4'd0;          // 1V..8V
logic [4:0] cnt_e7 = 5'd0;          // 16V..128V, [4] = VBLANK

always_ff @(posedge clk) begin
    if (pix) begin
        cnt_d5 <= cnt_d5 + 4'd1;
        if (cnt_d5 == 4'd15) begin
            cnt_e5 <= cnt_e5 + 5'd1;
            if (cnt_e5[3:0] == 4'd15 && !cnt_e5[4]) begin
                cnt_e5 <= 5'b11100;                          // HBLANK: reload 192, 320 counts per line
                cnt_e6 <= cnt_e6 + 4'd1;
                if (cnt_e6 == 4'd15) begin
                    cnt_e7 <= cnt_e7 + 5'd1;
                    if (cnt_e7[3:0] == 4'd15) begin
                        if (!cnt_e7[4]) begin
                            cnt_e6 <= 4'd10;                   // VBLANK: reload 218
                            cnt_e7 <= 5'b11101;
                        end else begin
                            cnt_e7 <= 5'b00010;                // active: 32
                        end
                    end
                end
            end
        end
    end
end

wire [8:0] hcnt = {cnt_e5, cnt_d5};                            // 0-255 active, 448-511 HBLANK
wire [7:0] vpos = {cnt_e7[3:0], cnt_e6};                       // 32-255 active, 218-255 during VBLANK
wire [8:0] hx   = cnt_e5[4] ? hcnt - 9'd192 : hcnt;            // 0-319
wire [8:0] vcnt = {cnt_e7, cnt_e6};

// ---------------------------------------------------------------- CPU

logic  [7:0] cpu_di;
logic  [7:0] cpu_do;
logic [15:0] cpu_a;
logic        cpu_sync, cpu_wr_n, cpu_inte;
logic        ready = 1'b0;
logic        cpu_int = 1'b0;

`ifdef VERILATOR
T8080se u_cpu
`else
T8080se #(.Mode(2), .T2Write(1)) u_cpu
`endif
(
    .RESET_n(rst_n),
    .CLK(clk),
    .CLKEN(cpu_ce),
    .READY(ready & ~pause),
    .HOLD(1'b0),
    .INT(cpu_int),
    .INTE(cpu_inte),
    .DBIN(),
    .SYNC(cpu_sync),
    .VAIT(),
    .HLDA(),
    .WR_n(cpu_wr_n),
    .A(cpu_a),
    .DI(cpu_di),
    .DO(cpu_do)
);

// status latch D7: [0] INTA, [4] OUT, [6] INP
logic [7:0] status = 8'd0;

// interrupt E3: set on (!64V & 128V) | VBLANK rising (V = 128 and 218), cleared by INTA
wire int_trig = (~cnt_e7[2] & cnt_e7[3]) | cnt_e7[4];
logic int_trig_d = 1'b0;

// RAM read register / READY: the CPU owns the RAM while 4H is high
logic [9:0] rr = 10'd0;
logic       asel_d = 1'b0;
wire        asel = cnt_d5[2];
logic [7:0] ram_q;
logic [7:0] rdb = 8'd0;             // RAM data one ce late, as on the 9.984 MHz board: the ASEL falling-edge
always_ff @(posedge clk) if (ce) rdb <= ram_q;   // latch still sees the CPU's byte

always_ff @(posedge clk) begin
    if (!rst_n) begin
        status     <= 8'd0;
        int_trig_d <= 1'b0;
        cpu_int    <= 1'b0;
        ready      <= 1'b0;
        rr         <= 10'd0;
        asel_d     <= 1'b0;
    end else if (ce) begin
        int_trig_d <= int_trig;
        if (status[0])
            cpu_int <= 1'b0;
        else if (!int_trig_d && int_trig)
            cpu_int <= cpu_inte;

        if (cpu_sync)
            status <= cpu_do;

        if (cpu_sync && cpu_a[13])
            ready <= 1'b0;
        else if (!ready)
            ready <= rr[9];

        if (cpu_sync && cpu_a[13])
            rr <= 10'd0;
        else if ((asel && !asel_d) || (!asel && asel_d && rr[8])) begin
            rr[7:0] <= rdb;
            rr[8]   <= 1'b1;
            rr[9]   <= rr[8];
        end
        asel_d <= asel;
    end
end

// ---------------------------------------------------------------- program ROM (0000-1FFF, 4000-5FFF)

logic [7:0] rom_q;
wire        rom_dl = ioctl_wr0 && ioctl_addr < 25'h8000 && !ioctl_addr[13];

dpram_dc #(.widthad_a(14)) u_rom
(
    .clock_a(clk),
    .address_a({ioctl_addr[14], ioctl_addr[12:0]}),
    .data_a(ioctl_dout),
    .wren_a(rom_dl),
    .q_a(),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b({cpu_a[14], cpu_a[12:0]}),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(rom_q),
    .byteena_b(1'b1)
);

// ---------------------------------------------------------------- shifter (MB14241) and I/O

logic [15:0] shift_data = 16'd0;
logic  [2:0] shift_cnt = 3'd0;
wire   [7:0] shift_q = shift_data[15 - shift_cnt -: 8];

logic [7:0] port_in;
always_comb begin
    case (cpu_a[1:0])
        2'd0:    port_in = in0;
        2'd1:    port_in = in1;
        2'd2:    port_in = in2;
        default: port_in = shift_q;
    endcase
end

// data in: INTA -> RST vector, INP -> ports, A13 -> RAM register, else ROM
always_comb begin
    if (status[0])      cpu_di = {3'b110, cnt_e7[2], ~cnt_e7[2], 3'b111};
    else if (status[6]) cpu_di = port_in;
    else if (cpu_a[13]) cpu_di = rr[7:0];
    else                cpu_di = rom_q;
end

wire  out_strobe = ~cpu_wr_n & status[4];
logic out_d = 1'b0;
wire  out_wr = ce & out_strobe & ~out_d;

logic [7:0] wd_cnt = 8'd0;
logic       v128_d = 1'b0;

always_ff @(posedge clk) begin
    if (!rst_n) begin
        shift_data <= 16'd0;
        shift_cnt  <= 3'd0;
        snd1       <= 8'd0;
        snd2       <= 8'd0;
        out_d      <= 1'b0;
    end else begin
        if (ce) out_d <= out_strobe;
        if (out_wr) begin
            case (cpu_a[2:0])
                3'd2: shift_cnt  <= cpu_do[2:0];
                3'd3: snd1       <= cpu_do;
                3'd4: shift_data <= {cpu_do, shift_data[15:8]};
                3'd5: snd2       <= cpu_do;
                default: ;
            endcase
        end
    end
end

// watchdog: 255 frames (128V) without a port 6 write resets the board
always_ff @(posedge clk) begin
    wd_reset <= 1'b0;
    if (reset) begin
        wd_cnt <= 8'd0;
    end else if (ce) begin
        v128_d <= cnt_e7[3];
        if (cnt_e7[3] && !v128_d && !pause)
            wd_cnt <= wd_cnt + 8'd1;
        if (out_wr && cpu_a[2:0] == 3'd6)
            wd_cnt <= 8'd0;
        if (wd_cnt == 8'd255) begin
            wd_reset <= 1'b1;
            wd_cnt   <= 8'd0;
        end
    end
end

// ---------------------------------------------------------------- video RAM (2000-3FFF, mirror 6000)

// Flip (cocktail / CRT Flip): the bitmap is scanned from the other corner so the picture matches MAME's
// flipped 260 x 224 image. Bytes are fetched one slot early and loaded 4 pixels earlier, bit-reversed.
wire flip = crt_flip ^ (cocktail & snd2[5]);

wire [4:0] col  = {cnt_e5[3:0], cnt_d5[3]};
wire [7:0] row  = flip ? 8'd31 - vpos : vpos;
wire [4:0] vcol = flip ? 5'd30 - col : col;

wire [12:0] ram_a  = asel ? cpu_a[12:0] : {row, vcol};
wire        ram_we = ce & ~cpu_wr_n & (rr[8] ^ rr[9]) & asel;

dpram_dc #(.widthad_a(13)) u_ram
(
    .clock_a(clk),
    .address_a(ram_a),
    .data_a(cpu_do),
    .wren_a(ram_we),
    .q_a(ram_q),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b(hs_address[12:0]),
    .data_b(hs_data_in),
    .wren_b(hs_write),
    .q_b(hs_data_out),
    .byteena_b(1'b1)
);

// ---------------------------------------------------------------- video shift register

logic [7:0] shift = 8'd0;
logic [7:0] hold  = 8'd0;
logic       vid   = 1'b0;

wire v_act     = ~cnt_e7[4];
wire load_norm = ~flip & v_act & ~cnt_e5[4] & (cnt_d5[2:0] == 3'd3);
wire load_flip =  flip & v_act & (cnt_d5[2:0] == 3'd7) & ((~cnt_e5[4] & hx != 9'd255) | hx == 9'd319);

always_ff @(posedge clk) begin
    if (pix) begin
        if (cnt_d5[2:0] == 3'd3)
            hold <= rdb;
        if (load_norm)
            shift <= rdb;
        else if (load_flip)
            shift <= {hold[0], hold[1], hold[2], hold[3], hold[4], hold[5], hold[6], hold[7]};
        else
            shift <= {1'b0, shift[7:1]};
        vid <= shift[0];
    end
end

// ---------------------------------------------------------------- sync / blank (MAME framing)

// Registered outputs show one count after their edge: bitmap pixel d is on screen at hx = d + 5, so the
// visible window hx 1-260 = 4 black pixels + 256 bitmap pixels; HSYNC = MAME H 272-287, VSYNC = V 236-239.
always_ff @(posedge clk) begin
    if (pix) begin
        if (hx == 9'd0)   video_hblank <= 1'b0;
        if (hx == 9'd260) begin
            video_hblank <= 1'b1;
            video_vblank <= cnt_e7[4];
        end
        if (hx == 9'd272) begin
            video_hs <= 1'b1;
            video_vs <= (vcnt >= 9'd486 && vcnt < 9'd490);
        end
        if (hx == 9'd288) video_hs <= 1'b0;
    end
end

assign video_r = {8{vid}};
assign video_g = {8{vid}};
assign video_b = {8{vid}};

assign audio = 16'sd0;

endmodule
