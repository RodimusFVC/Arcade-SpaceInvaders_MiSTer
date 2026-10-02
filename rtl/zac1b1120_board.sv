//============================================================================
//
//  Zaccaria 1B1120 board (The Invaders, Super Invader Attack, Dodgem)
//  Copyright (C) 2026 Rodimus
//
//  Memory map, inputs and object placement follow MAME zaccaria/zac1b1120.cpp (Mike Coates); the S2636 PVI object
//  logic follows MAME devices/machine/s2636.cpp (Vas Crabb). S2650 CPU core by DO (rtl/cpu/s2650).
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

// clk = 39.936 MHz. A fractional enable gives the board's master clock (15.625 MHz, Dodgem 14.318 MHz) on average;
// one output pixel per master clock: 908 x 312 total, 720 x 256 visible (MAME set_raw). The character background
// runs at master / 3 (240 visible pixels), the S2636 objects at master / 4 (227 PVI clocks per line). The S2650 runs
// at master / 16 with one bus cycle per 3 of its clocks: one ack every 48 master clocks.
// MRA index 0: program ROM 0000-17FF, character ROM (gfx1) at 2000-23FF.

module zac1b1120_board
(
    input  logic        clk,
    input  logic        reset,
    input  logic        pause,
    input  logic        dodgem,         // 14.318 MHz master, Dodgem inputs / no sound-board remap

    input  logic  [7:0] in0,            // 1E80 (bit 7 = background collision, from the board)
    input  logic  [7:0] in1,            // 1E81
    input  logic  [7:0] in2,            // 1E82
    input  logic  [7:0] in3,            // 1E85

    input  logic [24:0] ioctl_addr,
    input  logic  [7:0] ioctl_dout,
    input  logic        ioctl_wr0,

    input  logic        ov_en,          // colour overlay (MAME layout rectangles, raw x 0-719 / line 0-255)
    input  logic [1031:0] ov_tab,

    output logic        ce_pix,
    output logic        video_on,       // lit pixel
    output logic  [7:0] video_r,        // its colour (white, or the overlay's)
    output logic  [7:0] video_g,
    output logic  [7:0] video_b,
    output logic        video_hs,
    output logic        video_vs,
    output logic        video_hblank,
    output logic        video_vblank,

    output logic signed [15:0] audio
);

// ---------------------------------------------------------------- master clock enable

logic [16:0] frac = 17'd0;
wire  [16:0] step = dodgem ? 17'd14318 : 17'd15625;   // kHz, against 39936 kHz
wire  [17:0] fsum = {1'b0, frac} + {1'b0, step};
wire         mce  = fsum >= 18'd39936;
always_ff @(posedge clk) frac <= mce ? 17'(fsum - 18'd39936) : fsum[16:0];
assign ce_pix = mce;

// ---------------------------------------------------------------- raster

logic [9:0] h = 10'd0;              // 0-907
logic [8:0] v = 9'd0;               // 0-311
always_ff @(posedge clk) if (mce) begin
    if (h == 10'd907) begin
        h <= 10'd0;
        v <= (v == 9'd311) ? 9'd0 : v + 9'd1;
    end else
        h <= h + 10'd1;
end

wire vis_h = h < 10'd720;
wire vis_v = v < 9'd256;
always_ff @(posedge clk) if (mce) begin
    video_hblank <= ~vis_h;
    video_vblank <= ~vis_v;
    video_hs     <= h >= 10'd760 && h < 10'd828;
    video_vs     <= v >= 9'd272 && v < 9'd276;
end

// ---------------------------------------------------------------- S2650

wire        cpu_req, cpu_wr, cpu_mio, cpu_ene, cpu_dc;
wire [14:0] cpu_ad;
wire  [7:0] cpu_dw;
wire  [1:0] cpu_ph;
logic [7:0] cpu_dr;

logic [5:0] cdiv = 6'd0;            // 48 master clocks per bus cycle
always_ff @(posedge clk) if (mce) cdiv <= (cdiv == 6'd47) ? 6'd0 : cdiv + 6'd1;
wire ack = mce & (cdiv == 6'd47) & ~pause;

s2650_cpu u_cpu
(
    .clk(clk),
    .reset(reset),
    .req(cpu_req),
    .ack(ack),
    .ad(cpu_ad),
    .wr(cpu_wr),
    .dw(cpu_dw),
    .dr(cpu_dr),
    .mio(cpu_mio),
    .ene(cpu_ene),
    .dc(cpu_dc),
    .ph(cpu_ph),
    .irq(1'b0),
    .intack(),
    .ivec(8'h00),
    .sense(vis_v),                  // MAME: sense = !VBLANK
    .flag()
);

// The core's bus outputs are its combinational next state: between acks they hold the current access, during an ack
// they already show the next one. Registered copies therefore describe the access being completed at ack, and the
// core consumes dr at ack as that access's data (after reset: 0000 in the opcode state).
logic [14:0] a_r = 15'd0;
logic  [7:0] dw_r = 8'd0;
logic        wr_r = 1'b0, req_r = 1'b0, mio_r = 1'b1;
always_ff @(posedge clk) begin
    a_r   <= cpu_ad;
    dw_r  <= cpu_dw;
    wr_r  <= cpu_wr;
    req_r <= cpu_req;
    mio_r <= cpu_mio;
end
wire [12:0] ba  = a_r[12:0];
wire        bwr = ack & req_r & wr_r & mio_r;
wire        mio_l = mio_r;

wire [12:0] ra = ba;
wire is_rom  = ra < 13'h1800;
wire is_vram = ba[12:10] == 3'b110;                 // 1800-1BFF (write decode: the address being acknowledged)
wire is_ram  = ba[12:9] == 4'b1110;                 // 1C00-1DFF
wire is_io   = ba[12:8] == 5'h1E;                   // 1E80-1E86
wire is_pvi  = ba[12:8] == 5'h1F;                   // 1F00-1FFF
wire rd_vram = ra[12:10] == 3'b110;
wire rd_ram  = ra[12:9] == 4'b1110;
wire rd_io   = ra[12:8] == 5'h1E;
wire rd_pvi  = ra[12:8] == 5'h1F;

// ---------------------------------------------------------------- memories

logic [7:0] rom_q, gfx_q, vram_q, vram_vq, ram_q, pvi_q;
logic [9:0] vram_va;
logic [9:0] gfx_va;

dpram_dc #(.widthad_a(13)) u_rom
(
    .clock_a(clk), .address_a(ioctl_addr[12:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr < 25'h1800), .q_a(), .byteena_a(1'b1),
    .clock_b(clk), .address_b(ba), .data_b(8'd0), .wren_b(1'b0), .q_b(rom_q), .byteena_b(1'b1)
);

dpram_dc #(.widthad_a(10)) u_gfx
(
    .clock_a(clk), .address_a(ioctl_addr[9:0]), .data_a(ioctl_dout),
    .wren_a(ioctl_wr0 && ioctl_addr[24:10] == 15'h0008), .q_a(), .byteena_a(1'b1),
    .clock_b(clk), .address_b(gfx_va), .data_b(8'd0), .wren_b(1'b0), .q_b(gfx_q), .byteena_b(1'b1)
);

dpram_dc #(.widthad_a(10)) u_vram
(
    .clock_a(clk), .address_a(ba[9:0]), .data_a(dw_r),
    .wren_a(bwr & is_vram), .q_a(vram_q), .byteena_a(1'b1),
    .clock_b(clk), .address_b(vram_va), .data_b(8'd0), .wren_b(1'b0), .q_b(vram_vq), .byteena_b(1'b1)
);

dpram_dc #(.widthad_a(9)) u_ram
(
    .clock_a(clk), .address_a(ba[8:0]), .data_a(dw_r),
    .wren_a(bwr & is_ram), .q_a(ram_q), .byteena_a(1'b1),
    .clock_b(clk), .address_b(9'd0), .data_b(8'd0), .wren_b(1'b0), .q_b(), .byteena_b(1'b1)
);

dpram_dc #(.widthad_a(8)) u_pvi
(
    .clock_a(clk), .address_a(ba[7:0]), .data_a(dw_r),
    .wren_a(bwr & is_pvi), .q_a(pvi_q), .byteena_a(1'b1),
    .clock_b(clk), .address_b(8'd0), .data_b(8'd0), .wren_b(1'b0), .q_b(), .byteena_b(1'b1)
);

// PVI object registers the video needs every line, mirrored as they are written: 4 objects x 14 bytes, sizes, tone
logic [7:0] obj [4][14];
logic [7:0] obj_size = 8'd0, snd_period = 8'd0;
function automatic logic [7:0] obj_base(input int i);
    obj_base = i == 0 ? 8'h00 : i == 1 ? 8'h10 : i == 2 ? 8'h20 : 8'h40;
endfunction
always_ff @(posedge clk) if (bwr & is_pvi) begin
    for (int i = 0; i < 4; i++)
        if (ba[7:0] >= obj_base(i) && ba[7:0] < obj_base(i) + 8'd14) obj[i][ba[3:0]] <= dw_r;
    if (ba[7:0] == 8'hC0) obj_size   <= dw_r;
    if (ba[7:0] == 8'hC7) snd_period <= dw_r;
end

// ---------------------------------------------------------------- I/O, collisions

logic       bg_coll = 1'b0, bg_coll_acc = 1'b0;
logic [5:0] obj_coll = 6'd0, obj_coll_acc = 6'd0;   // CB bits 5-0: 1/2 1/3 1/4 2/3 2/4 3/4

logic [7:0] io_q;
always_comb begin
    case (ra[2:0])
        3'd0:    io_q = {~bg_coll, in0[6:0]};
        3'd1:    io_q = in1;
        3'd2:    io_q = in2;
        3'd5:    io_q = in3;
        3'd6:    io_q = dodgem ? 8'h01 : 8'hFF;   // Dodgem collision-detection cheat DIP off; else unused (active low)
        default: io_q = 8'hFF;
    endcase
end

always_comb begin
    if (!mio_l)                      cpu_dr = 8'h00;     // S2650 I/O space: nothing mapped (MAME reads 0)
    else if (is_rom)                 cpu_dr = rom_q;
    else if (rd_vram)                cpu_dr = vram_q;
    else if (rd_ram)                 cpu_dr = ram_q;
    else if (rd_io)                  cpu_dr = io_q;
    else if (rd_pvi && ra[7:0] == 8'hCB) cpu_dr = {2'b00, obj_coll};    // MAME: frame's object collisions
    else if (rd_pvi)                 cpu_dr = pvi_q;
    else                             cpu_dr = 8'hFF;
end

// sound board (rtl/zac_snd.sv): 74174 latch at 1E80 (bit 6 not on the board), S2636 tone into its filter.
// Dodgem's board is different and not documented: its SN76477 side stays disabled, the tone still plays.
logic [7:0] snd_l = 8'd0;
logic       pvi_tone = 1'b0;        // S2636 square wave (generated below)
always_ff @(posedge clk) begin
    if (reset)
        snd_l <= 8'd0;
    else if (bwr && is_io && ba[7:0] == 8'h80 && !dodgem)
        snd_l <= dw_r;
end

zac_snd u_snd
(
    .clk(clk),
    .reset(reset),
    .pause(pause),
    .latch(snd_l),
    .pvi(pvi_tone),
    .out(audio)
);

// ---------------------------------------------------------------- character background (master / 3)

logic [1:0] sub3 = 2'd0;
logic [8:0] bx = 9'd0;              // background pixel 0-302
always_ff @(posedge clk) if (mce) begin
    if (h == 10'd907) begin
        sub3 <= 2'd0;
        bx   <= 9'd0;
    end else if (sub3 == 2'd2) begin
        sub3 <= 2'd0;
        bx   <= bx + 9'd1;
    end else
        sub3 <= sub3 + 2'd1;
end

// tile of the next pixel's column: fetched a background pixel ahead, ready when its column starts
wire [8:0] bx_next = bx + 9'd1;
wire       lstart  = h == 10'd907;                  // column 0 loads as the line wraps
assign vram_va = {v[7:3], h >= 10'd900 ? 5'd0 : bx_next[7:3]};
assign gfx_va  = {vram_vq[6:0], v[2:0]};

logic [7:0] bg_shift = 8'd0;
always_ff @(posedge clk) if (mce) begin
    if (lstart || (sub3 == 2'd2 && bx_next[2:0] == 3'd0)) bg_shift <= gfx_q;     // a new tile starts next pixel
    else if (sub3 == 2'd2)                                bg_shift <= {bg_shift[6:0], 1'b0};
end
wire bg_pix = bg_shift[7];          // this master clock's background pixel, aligned with the object pixels

// ---------------------------------------------------------------- S2636 objects (master / 4)

// Per line (MAME s2636_device::render_next_line): count lines to each object, then show its 10 rows (scaled), then
// its duplicate VCB + 1 lines later at HCB. Placement as MAME zac1b1120: x = 4 * HC - 22 master clocks, first line
// VC + 1 (the counters latch one line before the first visible line).
logic [7:0] ocnt [4];
logic [3:0] odisp = 4'd0, odup = 4'd0;
logic [7:0] lbits [4];              // this line's object row
logic [10:0] lx [4];                // this line's object start (master clocks, signed)
logic [1:0] lsc [4];
logic [3:0] lon = 4'd0;

function automatic logic [1:0] oscale(input int i, input logic [7:0] sz);
    oscale = sz[2*i +: 2];
endfunction

always_ff @(posedge clk) begin
    if (reset) begin
        odisp <= 4'd0;
        odup  <= 4'd0;
        lon   <= 4'd0;
    end else if (mce && h == 10'd720) begin            // line set-up during horizontal blanking
        for (int i = 0; i < 4; i++) begin : obj_line
            logic [7:0] cnt, inc;
            logic       disp, dup;
            cnt  = ocnt[i];
            disp = odisp[i];
            dup  = odup[i];
            inc  = 8'd1 << (3 - oscale(i, obj_size));
            lon[i] <= 1'b0;
            if (v == 9'd311) begin                     // the line before the first visible one: latch
                cnt  = obj[i][12];
                disp = 1'b0;
                dup  = 1'b0;
            end else if (v < 9'd255) begin             // next line is visible
                if (cnt == 8'd0 && !disp) begin
                    cnt  = 8'd80;
                    disp = 1'b1;
                end
                if (disp) begin
                    cnt = cnt - inc;
                    lbits[i] <= obj[i][4'd9 - cnt[6:3]];          // cnt < 80 here: row 0-9
                    lx[i]    <= {1'b0, dup ? obj[i][11] : obj[i][10], 2'b00} - 11'd22;
                    lsc[i]   <= oscale(i, obj_size);
                    lon[i]   <= 1'b1;
                    if (cnt == 8'd0) begin
                        cnt  = 8'd1 + obj[i][13];
                        disp = 1'b0;
                        dup  = 1'b1;
                    end
                end else
                    cnt = cnt - 8'd1;
            end
            ocnt[i]  <= cnt;
            odisp[i] <= disp;
            odup[i]  <= dup;
        end
    end
end

// object pixels at this master clock (MSB of the row first, 4 << scale master clocks per bit)
logic [3:0] opix;
always_comb begin
    for (int i = 0; i < 4; i++) begin : obj_pix
        logic [10:0] rel;
        logic [10:0] idx;
        rel = {1'b0, h} - lx[i];
        idx = rel >> (2 + lsc[i]);                     // full width: a truncated index repeats the object
        opix[i] = lon[i] && vis_h && !rel[10] && idx < 11'd8 && lbits[i][3'd7 - idx[2:0]];
    end
end

// collisions, latched for the frame at the end of the visible area (MAME draw_sprites per frame)
always_ff @(posedge clk) if (mce) begin
    if (vis_v && vis_h) begin
        if (|opix && bg_pix) bg_coll_acc <= 1'b1;
        obj_coll_acc <= obj_coll_acc | {opix[0] & opix[1], opix[0] & opix[2], opix[0] & opix[3],
                                        opix[1] & opix[2], opix[1] & opix[3], opix[2] & opix[3]};
    end
    if (v == 9'd256 && h == 10'd0) begin
        bg_coll      <= bg_coll_acc;
        obj_coll     <= obj_coll_acc;
        bg_coll_acc  <= 1'b0;
        obj_coll_acc <= 6'd0;
    end
end

always_ff @(posedge clk) if (mce) video_on <= vis_h && vis_v && (bg_pix || |opix);

// colour overlay: rectangles as the 8080 board's table (x bits 8 / 9 in the flag byte, y1 0 = no bottom edge), later
// ones win; lit pixels take the colour
function automatic logic [7:0] ovb(input int i);
    ovb = ov_tab[i*8 +: 8];
endfunction

always_ff @(posedge clk) if (mce) begin : zov
    logic [7:0] r, g, b, hi;
    logic [9:0] x0, x1;
    {r, g, b} = 24'hFFFFFF;
    for (int k = 0; k < 16; k++) begin
        hi = ovb(1 + k*8 + 1);
        x0 = {hi[2], hi[0], ovb(1 + k*8)};
        x1 = {hi[3], hi[1], ovb(1 + k*8 + 2)};
        if (k < ovb(0) && ov_en && h >= x0 && h < x1 && v[7:0] >= ovb(1 + k*8 + 3) &&
            (v[7:0] < ovb(1 + k*8 + 4) || ovb(1 + k*8 + 4) == 8'd0))
            {r, g, b} = {ovb(1 + k*8 + 5), ovb(1 + k*8 + 6), ovb(1 + k*8 + 7)};
    end
    video_r <= (vis_h && vis_v && (bg_pix || |opix)) ? r : 8'd0;
    video_g <= (vis_h && vis_v && (bg_pix || |opix)) ? g : 8'd0;
    video_b <= (vis_h && vis_v && (bg_pix || |opix)) ? b : 8'd0;
end

// ---------------------------------------------------------------- S2636 tone: toggles every C7 + 1 lines

logic [7:0] tone_cnt = 8'd0;
always_ff @(posedge clk) begin
    if (reset)
        pvi_tone <= 1'b0;
    else if (mce && h == 10'd0) begin
        if (snd_period == 8'd0) begin
            pvi_tone <= 1'b0;
            tone_cnt <= 8'd0;
        end else if (tone_cnt >= snd_period) begin
            pvi_tone <= ~pvi_tone;
            tone_cnt <= 8'd0;
        end else
            tone_cnt <= tone_cnt + 8'd1;
    end
end

endmodule
