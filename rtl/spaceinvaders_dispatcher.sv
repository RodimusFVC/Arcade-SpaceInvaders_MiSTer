//============================================================================
//  spaceinvaders_dispatcher - Space Invaders shared pools and board select
//  Copyright (C) 2026 Rodimus
//
//  Two boards, one running at a time: the Midway / Taito 8080 board (rtl/boards/mw8080) and the Zaccaria 1B1120
//  board (rtl/boards/zac1b1120, variant 32 / 33). Their CPUs, EEPROM, sound engines, ROM store and board RAMs live
//  here once; the boards drive the pool ports (Pool contract in the core's Claude/ note).
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

module spaceinvaders_dispatcher
(
    input  logic        clk,
    input  logic        reset,
    input  logic        pause,
    input  logic  [7:0] variant,        // MRA index 1 byte 0: 32 / 33 = Zaccaria (33 Dodgem), else the 8080 board
    input  logic  [7:0] snd_flags,      // byte 2: sound board value, [3] the dsnd program is an invaders board
    input  logic  [2:0] rom_dec,        // byte 3
    input  logic [31:0] f3,             // bytes 12-15
    input  logic [127:0] io_tab,        // bytes 48-63
    input  logic [1031:0] ov_tab,       // bytes 64+: colour overlay
    input  logic        ov_en,

    input  logic  [7:0] in0,
    input  logic  [7:0] in1,
    input  logic  [7:0] in2,
    input  logic  [7:0] in3,
    input  logic  [7:0] gun_x,
    input  logic  [7:0] gun_y,
    input  logic        gun_trig,
    input  logic        xhair_en,

    input  logic        crt_flip,
    input  logic signed [3:0] h_adj,
    input  logic signed [3:0] v_adj,
    input  logic [13:0] game_audio,     // Game Audio {music, aged caps, oscillator, filter, timing} (sources 6 / 7)
    input  logic        ds_vblank,      // VBLANK as the core's video path shows it (engine source 5 bit 0)

    input  logic        ioctl_download,
    input  logic  [7:0] ioctl_index,
    input  logic        ioctl_wr,
    input  logic [24:0] ioctl_addr,
    input  logic  [7:0] ioctl_dout,

    output logic        ce_pix,
    output logic  [7:0] video_r,
    output logic  [7:0] video_g,
    output logic  [7:0] video_b,
    output logic        video_hs,
    output logic        video_vs,
    output logic        video_hblank,
    output logic        video_vblank,

    output logic signed [15:0] audio_l,
    output logic signed [15:0] audio_r, // second sound board (invad2ct), else the same as audio_l
    output logic [31:0] snd_lat,        // 8080 board sound latches 4-1 (the top's analog select line)

    input  logic [15:0] hs_address,
    input  logic  [7:0] hs_data_in,
    output logic  [7:0] hs_data_out,
    input  logic        hs_write
);

wire zac = variant >= 8'd32;

/////////////////////////////  Shared CPU / sound chip pool  /////////////////////////////

// t8080se_0: the 8080 board's CPU (every set but sflush, which holds it in reset)
chip_pool_pkg::t8080se_i_t t8080se_0_i;
chip_pool_pkg::t8080se_o_t t8080se_0_o;

t80sv_T8080se #(.Mode(2), .T2Write(1)) t8080se_0
(
    .RESET_n(t8080se_0_i.reset_n),
    .CLK(clk),
    .CLKEN(t8080se_0_i.clken),
    .READY(t8080se_0_i.ready),
    .HOLD(t8080se_0_i.hold),
    .INT(t8080se_0_i.intr),
    .INTE(t8080se_0_o.inte),
    .DBIN(t8080se_0_o.dbin),
    .SYNC(t8080se_0_o.sync),
    .VAIT(t8080se_0_o.vait),
    .HLDA(t8080se_0_o.hlda),
    .WR_n(t8080se_0_o.wr_n),
    .A(t8080se_0_o.a),
    .DI(t8080se_0_i.di),
    .DO(t8080se_0_o.dout)
);

// s2650_0: the Zaccaria board's CPU
chip_pool_pkg::s2650_i_t s2650_0_i;
chip_pool_pkg::s2650_o_t s2650_0_o;

s2650sv_s2650_cpu s2650_0
(
    .clk(clk),
    .reset(s2650_0_i.reset),
    .req(s2650_0_o.req),
    .ack(s2650_0_i.ack),
    .ad(s2650_0_o.ad),
    .wr(s2650_0_o.wr),
    .dw(s2650_0_o.dw),
    .dr(s2650_0_i.dr),
    .mio(s2650_0_o.mio),
    .ene(s2650_0_o.ene),
    .dc(s2650_0_o.dc),
    .ph(s2650_0_o.ph),
    .irq(s2650_0_i.irq),
    .intack(s2650_0_o.intack),
    .ivec(8'h00),
    .sense(s2650_0_i.sense),
    .flag()
);

// eeprom93c46_0: invmulti's 93C46 (8080 board)
chip_pool_pkg::eeprom93c46_i_t eeprom93c46_0_i;
chip_pool_pkg::eeprom93c46_o_t eeprom93c46_0_o;

eeprom_93c46 eeprom93c46_0
(
    .clk(clk),
    .reset(eeprom93c46_0_i.reset),
    .clear(eeprom93c46_0_i.clear),
    .wr(eeprom93c46_0_i.wr),
    .di(eeprom93c46_0_i.di),
    .sk(eeprom93c46_0_i.sk),
    .cs(eeprom93c46_0_i.cs),
    .dout(eeprom93c46_0_o.dout),
    .ext_sel(eeprom93c46_0_i.ext_sel),
    .ext_addr(eeprom93c46_0_i.ext_addr),
    .ext_din(eeprom93c46_0_i.ext_din),
    .ext_we(eeprom93c46_0_i.ext_we),
    .ext_q(eeprom93c46_0_o.ext_q)
);

// dsnd_0 / dsnd_1: the discrete sound engine (tools/dsnd_boards.py programs, MRA index 5). Engine 1 = index 5 from 0,
// engine 2 (invad2ct's second board, right channel) = index 5 from 0x2000. A program is loaded from the first index 5
// byte until the next set's index 1.
logic ds_on = 1'b0, ds2_on = 1'b0;
always_ff @(posedge clk) if (ioctl_wr) begin
    if (ioctl_index == 8'd1 && ioctl_addr == 25'd0) begin ds_on <= 1'b0; ds2_on <= 1'b0; end
    if (ioctl_index == 8'd5) ds_on <= 1'b1;
    if (ioctl_index == 8'd5 && ioctl_addr[13]) ds2_on <= 1'b1;
end

// sources: 7-6 Game Audio, 5 misc {Zaccaria tone bit, coin (IN1 D3 inverted, maze), VBLANK}, 4 = IN0, 3-0 latches.
// 8080 board: 1 / 2 = sound ports 1 / 2, 3 = port 0 / polaris port 2, 4 = last shift-data write, ozmawars port 4; an
// invaders board program (snd_flags[3]): latches 1 / 2 = the board's inputs after its per-set remaps.
// Zaccaria: 1 = the 1E80 latch, 2 = the S2636 tone period.
logic  [7:0] b8_snd1, b8_snd2, b8_dsp1, b8_dsp2, z_dslat, z_dsper;
logic [15:0] b8_snd34;
logic        z_dstone;
logic signed [16:0] b8_tune;
assign snd_lat = {b8_snd34, b8_snd2, b8_snd1};

wire [31:0] ds_lat = zac ? {16'd0, z_dsper, z_dslat} : snd_flags[3] ? {b8_snd34, b8_dsp2, b8_dsp1} : snd_lat;
chip_pool_pkg::dsnd_i_t dsnd_0_i, dsnd_1_i;
chip_pool_pkg::dsnd_o_t dsnd_0_o, dsnd_1_o;
assign dsnd_0_i.src = {game_audio[13:6], 2'd0, game_audio[5:0], 5'd0, zac & z_dstone, ~in1[3], ds_vblank, in0, ds_lat};
assign dsnd_1_i.src = {dsnd_0_i.src[63:16], b8_snd34};           // invad2ct board 2: latches 3 / 4 as 1 / 2

dsnd_engine dsnd_0
(
    .clk(clk),
    .reset(reset),
    .pause(pause),
    .prog_wr(ioctl_wr && ioctl_index == 8'd5 && !ioctl_addr[13]),
    .prog_addr(ioctl_addr[12:0]),
    .prog_data(ioctl_dout),
    .src(dsnd_0_i.src),
    .out(dsnd_0_o.out),
    .out_r(dsnd_0_o.out_r),
    .sample(dsnd_0_o.sample)
);

dsnd_engine dsnd_1
(
    .clk(clk),
    .reset(reset),
    .pause(pause),
    .prog_wr(ioctl_wr && ioctl_index == 8'd5 && ioctl_addr[13]),
    .prog_addr(ioctl_addr[12:0]),
    .prog_data(ioctl_dout),
    .src(dsnd_1_i.src),
    .out(dsnd_1_o.out),
    .out_r(dsnd_1_o.out_r),
    .sample(dsnd_1_o.sample)
);

/////////////////////////////  Shared ROM blocks  /////////////////////////////

// rom0: MRA index 0 0x00000-0x1FFFF (128K), stored as loaded for both boards (the variant arrives after the ROMs).
// 8080 board: CPU program on port B; port A = high-nibble chips (spaceattbp) / invrvnge sound ROMs.
// Zaccaria board: program ROM 0000-17FF on port B; port A = character ROM 2000-23FF.
// Port A loads the block during download and is the second read port otherwise.
wire        load_rom0 = ioctl_wr && ioctl_index == 8'd0 && ioctl_addr < 25'h20000;
logic [16:0] b8_r0a, b8_r0b_a, z_r0a, z_r0b_a;
logic  [7:0] rom0_d, rom0b_d;

dpram_dc #(.widthad_a(17)) rom0
(
    .clock_a(clk), .address_a(load_rom0 ? ioctl_addr[16:0] : zac ? z_r0b_a : b8_r0b_a), .data_a(ioctl_dout),
    .wren_a(load_rom0), .q_a(rom0b_d), .byteena_a(1'b1),
    .clock_b(clk), .address_b(zac ? z_r0a : b8_r0a), .data_b(8'd0), .wren_b(1'b0), .q_b(rom0_d), .byteena_b(1'b1)
);

/////////////////////////////  Shared RAM pool  /////////////////////////////

//   block 0 (8K)  8080: video RAM (port B = hiscore / sflush CPU)   Zaccaria: video RAM (1K)
//   block 1 (8K)  8080: vortex colour copy / sflush work RAM        Zaccaria: work RAM (512)
//   block 2 (4K)  8080: colour PROMs + polaris clouds (loaded from index 0 at 0x20000)   Zaccaria: S2636 registers (256)
//   block 3 (2K)  8080: darthvdr work RAM
//   block 4 (2K)  8080: colour RAM
//   block 5 (1K)  8080: second colour RAM (rollingc)
//   block 6 (128) 8080: invrvnge sound CPU RAM
// The 8080 board owns the pool during a ROM download: block 2 loads then, before the new set's variant arrives.
localparam int RAM_BLOCKS = 7;

ram_pool_pkg::pram_i_t [RAM_BLOCKS-1:0] b8_pr_i, z_pr_i, pool_i;
ram_pool_pkg::pram_o_t [RAM_BLOCKS-1:0] pool_o;

assign pool_i = zac && !(ioctl_download && ioctl_index == 8'd0) ? z_pr_i : b8_pr_i;

function automatic int ram_aw(input int n);
    ram_aw = n < 2 ? 13 : n == 2 ? 12 : n < 5 ? 11 : n == 5 ? 10 : 7;
endfunction

genvar pn;
generate
    for (pn = 0; pn < RAM_BLOCKS; pn = pn + 1) begin : rpool
        localparam int AW = ram_aw(pn);
        dpram_dc #(.widthad_a(AW)) ram
        (
            .clock_a(clk), .address_a(pool_i[pn].a_addr[AW-1:0]), .data_a(pool_i[pn].a_d), .wren_a(pool_i[pn].a_we),
            .q_a(pool_o[pn].a_q), .byteena_a(1'b1),
            .clock_b(clk), .address_b(pool_i[pn].b_addr[AW-1:0]), .data_b(pool_i[pn].b_d), .wren_b(pool_i[pn].b_we),
            .q_b(pool_o[pn].b_q), .byteena_b(1'b1)
        );
    end
endgenerate

/////////////////////////////  Boards  /////////////////////////////

logic        b8_ce, b8_hs, b8_vs, b8_hb, b8_vb;
logic  [7:0] b8_r, b8_g, b8_b;
logic signed [15:0] b8_audio, b8_audio_r;

mw8080_board board
(
    .clk(clk),
    .reset(reset | zac),
    .pause(pause),
    .variant(variant),

    .in0(in0),
    .in1(in1),
    .in2(in2),
    .in3(in3),
    .cocktail(1'b0),
    .rom_dec(rom_dec),
    .gun_x(gun_x),
    .gun_y(gun_y),
    .gun_trig(gun_trig),
    .xhair_en(xhair_en),
    .f3(f3),

    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_dout),
    .ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),

    .rom0_a(b8_r0a),
    .rom0_d(rom0_d),
    .rom0b_a(b8_r0b_a),
    .rom0b_d(rom0b_d),

    .crt_flip(crt_flip),
    .h_adj(h_adj),
    .v_adj(v_adj),
    .ov_en(ov_en),
    .ov_tab(ov_tab),

    .ce_pix(b8_ce),
    .video_r(b8_r),
    .video_g(b8_g),
    .video_b(b8_b),
    .video_hs(b8_hs),
    .video_vs(b8_vs),
    .video_hblank(b8_hb),
    .video_vblank(b8_vb),

    .snd1(b8_snd1),
    .snd2(b8_snd2),
    .snd34(b8_snd34),
    .audio(b8_audio),
    .audio_r(b8_audio_r),
    .ds_p1(b8_dsp1),
    .ds_p2(b8_dsp2),
    .ds_tune(b8_tune),
    .io_tab(io_tab),

    .hs_address(hs_address),
    .hs_data_in(hs_data_in),
    .hs_data_out(hs_data_out),
    .hs_write(hs_write),

    .t8080se_0_i(t8080se_0_i),
    .t8080se_0_o(t8080se_0_o),
    .eeprom93c46_0_i(eeprom93c46_0_i),
    .eeprom93c46_0_o(eeprom93c46_0_o),
    .pr_i(b8_pr_i),
    .pr_o(pool_o)
);

logic        z_ce, z_hs, z_vs, z_hb, z_vb;
logic  [7:0] z_r, z_g, z_b;

zac1b1120_board zboard
(
    .clk(clk),
    .reset(reset | ~zac),
    .pause(pause),
    .dodgem(variant == 8'd33),

    .in0(in0),
    .in1(in1),
    .in2(in2),
    .in3(in3),

    .rom0_a(z_r0a),
    .rom0_d(rom0_d),
    .rom0b_a(z_r0b_a),
    .rom0b_d(rom0b_d),

    .h_adj(h_adj),
    .v_adj(v_adj),
    .ov_en(ov_en),
    .ov_tab(ov_tab),

    .ce_pix(z_ce),
    .video_on(),
    .video_r(z_r),
    .video_g(z_g),
    .video_b(z_b),
    .video_hs(z_hs),
    .video_vs(z_vs),
    .video_hblank(z_hb),
    .video_vblank(z_vb),

    .ds_latch(z_dslat),
    .ds_period(z_dsper),
    .ds_tone(z_dstone),

    .s2650_0_i(s2650_0_i),
    .s2650_0_o(s2650_0_o),
    .pr_i(z_pr_i),
    .pr_o(pool_o)
);

assign {video_hs, video_vs, video_hblank, video_vblank, ce_pix} = zac ? {z_hs, z_vs, z_hb, z_vb, z_ce} :
                                                                        {b8_hs, b8_vs, b8_hb, b8_vb, b8_ce};
assign {video_r, video_g, video_b} = zac ? {z_r, z_g, z_b} : {b8_r, b8_g, b8_b};

// audio: an invaders board program carries the board's tune speaker on the engine output, as the board's own mix does
wire signed [15:0] ds_audio  = dsnd_0_o.out;
wire signed [15:0] ds2_audio = dsnd_1_o.out;
wire signed [16:0] ds_sum = ds_audio + (snd_flags[3] ? b8_tune : 17'sd0);
wire signed [15:0] ds_out = ds_sum > 17'sd32767 ? 16'sd32767 : ds_sum < -17'sd32768 ? -16'sd32768 : 16'(ds_sum);
assign audio_l = ds_on ? ds_out : zac ? 16'sd0 : b8_audio;
assign audio_r = zac ? audio_l : ds_on ? (ds2_on ? ds2_audio : ds_out) : b8_audio_r;

endmodule
