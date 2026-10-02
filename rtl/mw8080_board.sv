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
// variant (MRA index 1 byte 0): 0 invaders (MB14241 shifter on port 3), 1 no shifter (port 3 open),
//   2 invasion (port 3 = IN3), 3 spcewars (port 3 bit 4 = 1-bit tune speaker, no sound enable, no watchdog),
//   4 yosakdon (no shifter, own sound bits, no watchdog), 5 darthvdr (ROM 0000-17FF, work RAM 1800-1FFF,
//   video RAM 4000-5FFF, vblank IRQ only with RST 7, port 0 flip / port 8 sound, no watchdog),
//   6 attackfc (shifter data 3 / count 7, no sound, no watchdog), 7 vortex (I/O port A1 inverted, colour from RAM
//   bit 0 of the next byte), 8 spacecom (one-hot ports 41/42/44, no shifter, no watchdog, 256-pixel picture),
//   9 invmulti (128K scrambled ROM banked into 0000/4000 by writes to E000, RAM 2000 only, 93C46 EEPROM at 6000),
//   10 shuttlei (256 x 192 MSB-first bitmap from 2000, 1K RAM at 4000 / 6000, ports FD sound, FE DSW / sound
//   code, FF inputs / flip, no shifter, no watchdog), 11 claybust (light gun: trigger latches the RAM address under
//   the gun on ports 2 / 6, IN1 [0] gun on 250 ms, [1] trigger 2 frames; shifter count 1 / data 2, watchdog 4, silent)
//   12 starw1 (no shifter, sound on ports 4 / 5), 13 spcewarla (shifter count 8 / data C / result 6, sound 4 / 5),
//   14 polaris (port 0 = IN0 [2:0] + IN1 [7:3], music 2, sound 4 / 6, watchdog 5), 15 indianbt (port 0 protection
//   read keyed on the IN instruction's address, as MAME), 16 schasercv (port 2 = IN2 + IN1 controls), 18 cane
//   (claybust ports without the gun), 19 orbite (ports 06 / 08 / 20 / 40 / 66 / 76 / 7A), 20 Midway one-offs:
//   table-driven I/O (io_tab = MRA index 1 bytes 48-63, tools/gen_mra.py FAMILY4)
// f3 (MRA index 1 bytes 12-15, family 3 colour boards, tools/gen_mra.py FAMILY3): [7:0] colour mode, [15:8] colour
//   flags, [23:16] sound map, [24] no watchdog, [26:25] variant 20 watchdog (255 frames / 2.97 s / 1.1 s / none),
//   [27] variant 20 decodes A3 (ports 8-15 open)
// rom_dec (MRA index 1 byte 3): [0] ROM A8/A9 swapped (attackfc), [1] ROM A0/A3/A9 inverted (vortex),
//   [2] 4-bit bproms: low nibbles at the CPU address, high nibbles 0x10000 above (spaceattbp)

module mw8080_board
(
    input  logic        clk,
    input  logic        reset,
    input  logic        pause,
    input  logic  [7:0] variant,

    input  logic  [7:0] in0,
    input  logic  [7:0] in1,
    input  logic  [7:0] in2,
    input  logic  [7:0] in3,
    input  logic        cocktail,
    input  logic        taito_snd,      // Taito L-shaped sound board (else Midway)
    input  logic  [2:0] rom_dec,
    input  logic  [7:0] gun_x,          // light gun: bitmap pixel 0-255
    input  logic  [7:0] gun_y,          //            picture row 0-223
    input  logic        gun_trig,
    input  logic        xhair_en,       // draw the crosshair
    input  logic [31:0] f3,
    input  logic [127:0] io_tab,        // variant 20: [63:0] read source per port, [127:64] write targets per port

    input  logic [24:0] ioctl_addr,
    input  logic  [7:0] ioctl_dout,
    input  logic        ioctl_wr0,

    input  logic        crt_flip,
    input  logic        ov_en,          // colour overlay on
    input  logic [1031:0] ov_tab,       // overlay: byte 0 count, then 8 bytes per rectangle (MRA index 1 from 64)

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
    output logic signed [15:0] audio_r,

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

wire v_dvdr  = variant == 8'd5;
wire v_afc   = variant == 8'd6;
wire v_vtx   = variant == 8'd7;
wire v_scom  = variant == 8'd8;
wire v_imul  = variant == 8'd9;
wire v_shut  = variant == 8'd10;
wire v_clay  = variant == 8'd11;
wire v_sw1   = variant == 8'd12;
wire v_swla  = variant == 8'd13;
wire v_pol   = variant == 8'd14;
wire v_ind   = variant == 8'd15;
wire v_scv   = variant == 8'd16;
wire v_cane  = variant == 8'd18;
wire v_orb   = variant == 8'd19;
wire v_tab   = variant == 8'd20;

wire [7:0] col_mode = f3[7:0];       // 0 none, 1 PROM, 2 colour RAM C000, 3 schaser, 4 polaris, 5 rollingc, 6 cosmo
wire [7:0] cflags   = f3[15:8];
wire [7:0] smap     = f3[23:16];
wire       colour   = col_mode != 8'd0 && col_mode != 8'd7;      // 7 = phantom2 clouds on the B&W picture
wire is_cram  = ((col_mode == 8'd2 || col_mode == 8'd3 || col_mode == 8'd4) && cpu_a[15:13] == 3'b110) |
                (col_mode == 8'd5 && cpu_a[15:13] == 3'b101) | (col_mode == 8'd6 && cpu_a[15:10] == 6'b010111);
wire is_cram2 = col_mode == 8'd5 && cpu_a[15:13] == 3'b111;          // rollingc background colours
wire is_star  = col_mode == 8'd6 && cpu_a[15:10] == 6'b010110;       // cosmo 5800 star control (not modelled)
logic [7:0] cram_qa, cram_qb, cram2_qa, cram2_qb;
wire narrow  = v_scom | v_shut;                                   // 256-pixel picture without the 4-pixel delay
wire wd_en   = (variant <= 8'd2 || v_vtx || v_imul || v_clay || v_sw1 || v_swla || v_pol || v_ind || v_cane || v_orb ||
                (v_tab && f3[26:25] != 2'd3)) && !f3[24];         // boards with a watchdog
wire [7:0] wd_len = !v_tab ? 8'd255 : f3[26:25] == 2'd1 ? 8'd178 : f3[26:25] == 2'd2 ? 8'd66 : 8'd255;
wire is_ram  = v_dvdr ? cpu_a[15:13] == 3'b010 :                  // the shared (video) DRAM
               v_imul ? cpu_a[14:13] == 2'b01 :
               v_shut ? cpu_a[15:13] == 3'b001 : cpu_a[13] & ~is_cram & ~is_cram2;
wire is_eep  = v_imul & cpu_a[15:13] == 3'b011;                   // invmulti EEPROM 6000-7FFF
wire is_bank = v_imul & cpu_a[15:13] == 3'b111;                   // invmulti bank latch E000-FFFF (write only)
wire is_wram = (v_dvdr & cpu_a[15:11] == 5'b00011) |              // darthvdr work RAM 1800-1FFF
               (v_shut & (cpu_a[15:10] == 6'b010000 | cpu_a[15:10] == 6'b011000));   // shuttlei 4000 / skylove 6000

// address of the last opcode fetch (indianbt's port 0 read depends on it) and a free-running random byte
logic [15:0] pc_m1 = 16'd0;
logic [15:0] lfsr = 16'hACE1;
wire  [7:0]  rnd = lfsr[7:0];
always_ff @(posedge clk) begin
    lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
    if (ce && cpu_sync && cpu_do[5]) pc_m1 <= cpu_a;
end

// interrupt E3: set on (!64V & 128V) | VBLANK rising (V = 128 and 218), cleared by INTA; darthvdr: VBLANK only
wire int_trig = v_dvdr ? cnt_e7[4] : (~cnt_e7[2] & cnt_e7[3]) | cnt_e7[4];
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

        if (cpu_sync && is_ram)
            ready <= 1'b0;
        else if (!ready)
            ready <= rr[9];

        if (cpu_sync && is_ram)
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

// MRA index 0 is stored as loaded (up to 128K); every set reads its CPU windows at their own offsets, invmulti
// reads its bank through MAME init_invmulti's address / data scramble.
logic [7:0] rom_raw, rom_hi;
logic [2:0] bank = 3'd0;
wire [12:0] rom_a = rom_dec[0] ? {cpu_a[12:10], cpu_a[8], cpu_a[9], cpu_a[7:0]} :   // decrypted on the read path
                    rom_dec[1] ? cpu_a[12:0] ^ 13'h0209 :
                                 cpu_a[12:0];
wire [16:0] im_i  = {bank, cpu_a[14], cpu_a[12:0]};
wire [16:0] rom_ra = v_imul ? {im_i[16], im_i[15], im_i[11], im_i[12], im_i[13], im_i[8], im_i[14], im_i[9], im_i[10],
                               im_i[7:0]} :
                              {2'b00, cpu_a[14], 1'b0, rom_a};
wire  [7:0] rom_q = v_imul     ? {rom_raw[0], rom_raw[6], rom_raw[5], rom_raw[7], rom_raw[4], rom_raw[3], rom_raw[1], rom_raw[2]} :
                   rom_dec[2] ? {rom_hi[3:0], rom_raw[3:0]} :
                                rom_raw;
wire        rom_dl = ioctl_wr0 && ioctl_addr < 25'h20000;

dpram_dc #(.widthad_a(17)) u_rom
(
    .clock_a(clk),
    .address_a(reset ? ioctl_addr[16:0] : rom_ra | 17'h10000),   // download port; at run time the high-nibble chip
    .data_a(ioctl_dout),
    .wren_a(rom_dl),
    .q_a(rom_hi),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b(rom_ra),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(rom_raw),
    .byteena_b(1'b1)
);

// ---------------------------------------------------------------- shifter (MB14241) and I/O

logic [15:0] shift_data = 16'd0;
logic  [2:0] shift_cnt = 3'd0;
wire   [7:0] shift_q = shift_data[15 - shift_cnt -: 8];
wire   [7:0] shift_rev = {shift_q[0], shift_q[1], shift_q[2], shift_q[3], shift_q[4], shift_q[5], shift_q[6], shift_q[7]};

// port number as the invaders map sees it: vortex inverts A1; spacecom decodes ports 41 / 42 / 44 one-hot
// claybust writes: 1 shift count, 2 shift data, 4 watchdog
// variant 20: port tables (A3 high reads open / writes nothing when the board decodes A3)
wire       tab_off = f3[27] & cpu_a[3];
wire [7:0] tab_rd  = tab_off ? 8'd0 : io_tab[cpu_a[2:0]*8 +: 8];
wire [7:0] tab_wr  = tab_off ? 8'd0 : io_tab[64 + cpu_a[2:0]*8 +: 8];
logic      rev_sh  = 1'b0;          // reversible shifter (boothill): shift count bit 3 selects the reversed result

wire [2:0] io_a = v_tab  ? 3'd7 :
                  v_vtx  ? cpu_a[2:0] ^ 3'b010 :
                  v_scom ? (cpu_a[0] ? 3'd1 : cpu_a[1] ? 3'd2 : cpu_a[2] ? 3'd3 : 3'd0) :
                  v_clay | v_cane ? (cpu_a[2:0] == 3'd1 ? 3'd2 : cpu_a[2:0] == 3'd2 ? 3'd4 : cpu_a[2:0] == 3'd4 ? 3'd6 :
                                     v_cane && cpu_a[2:0] == 3'd3 ? 3'd3 : v_cane && cpu_a[2:0] == 3'd5 ? 3'd5 : 3'd7) :
                  v_sw1  ? (cpu_a[2:0] == 3'd4 ? 3'd3 : cpu_a[2:0] == 3'd5 ? 3'd5 : cpu_a[2:0] == 3'd6 ? 3'd6 : 3'd7) :
                  v_swla ? (cpu_a[3:0] == 4'h4 ? 3'd3 : cpu_a[3:0] == 4'h5 ? 3'd5 : cpu_a[3:0] == 4'h6 ? 3'd6 :
                            cpu_a[3:0] == 4'h8 ? 3'd2 : cpu_a[3:0] == 4'hC ? 3'd4 : 3'd7) :
                  v_pol  ? (cpu_a[2:0] == 3'd0 ? 3'd2 : cpu_a[2:0] == 3'd3 ? 3'd4 : cpu_a[2:0] == 3'd4 ? 3'd3 :
                            cpu_a[2:0] == 3'd5 ? 3'd6 : cpu_a[2:0] == 3'd6 ? 3'd5 : cpu_a[2:0] == 3'd2 ? 3'd0 : 3'd7) :
                  v_orb  ? (cpu_a[7:0] == 8'h06 ? 3'd6 : cpu_a[7:0] == 8'h20 ? 3'd2 : cpu_a[7:0] == 8'h40 ? 3'd4 : 3'd7) :
                           cpu_a[2:0];

// claybust light gun (MAME claybust_state): the trigger latches ((x >> 3) | (y << 5)) + 2 with y = V 0x20-0xFF,
// "gun on" holds for 250 ms, the trigger bit is a 2-frame pulse
logic [15:0] gun_pos = 16'd0;
logic [21:0] gun_tim = 22'd0;
logic  [1:0] gun_imp = 2'd0;
logic        trig_d = 1'b0, vb_d = 1'b0;
always_ff @(posedge clk) begin
    if (!rst_n) begin
        gun_pos <= 16'd0;
        gun_tim <= 22'd0;
        gun_imp <= 2'd0;
    end else if (ce) begin
        trig_d <= gun_trig;
        vb_d   <= cnt_e7[4];
        if (cnt_e7[4] && !vb_d && gun_imp != 2'd0) gun_imp <= gun_imp - 2'd1;
        if (gun_tim != 22'd0) gun_tim <= gun_tim - 22'd1;
        if (gun_tim == 22'd1) gun_pos <= 16'd0;
        if (gun_trig && !trig_d && v_clay) begin
            gun_pos <= {3'd0, gun_y + 8'd32, gun_x[7:3]} + 16'd2;
            gun_tim <= 22'd2496000;
            gun_imp <= 2'd2;
        end
    end
end

logic       sh_flip = 1'b0;

logic [7:0] port_in;
always_comb begin
    if (v_tab)
        case (tab_rd[3:0])
            4'd1: port_in = in0;
            4'd2: port_in = in1;
            4'd3: port_in = in2;
            4'd4: port_in = in3;
            4'd5: port_in = shift_q;
            4'd6: port_in = shift_rev;
            4'd7: port_in = ~shift_q;
            4'd8: port_in = rev_sh ? shift_rev : shift_q;
            default: port_in = 8'h00;
        endcase
    else if (v_sw1)
        port_in = cpu_a[2:0] == 3'd1 ? in1 : cpu_a[2:0] == 3'd2 ? in2 : 8'h00;
    else if (v_swla)
        port_in = cpu_a[3:0] == 4'h0 ? in0 : cpu_a[3:0] == 4'h1 ? in1 : cpu_a[3:0] == 4'h2 ? in2 :
                  cpu_a[3:0] == 4'h6 ? shift_q : 8'h00;
    else if (v_pol && cpu_a[1:0] == 2'd0)
        port_in = {in1[7:3], in0[2:0]};                           // upright: MAME polaris_port00_r
    else if (v_ind && cpu_a[2:0] == 3'd0)                         // MAME indianbt_r: keyed on the IN's PC
        port_in = pc_m1 == 16'h5FEB ? 8'h10 : pc_m1 == 16'h5FFA ? 8'h00 : rnd;
    else if (v_scv && cpu_a[1:0] == 2'd2)                         // upright: MAME schasercv_02_r
        port_in = (in2 & 8'h89) | (in1 & 8'h70) | {5'd0, in1[7], in1[3], 1'b0};
    else if (v_cane)
        port_in = cpu_a[2:0] == 3'd1 ? in1 : cpu_a[2:0] == 3'd3 ? shift_q : 8'h00;
    else if (v_orb)
        port_in = cpu_a[7:0] == 8'h08 ? shift_q : cpu_a[7:0] == 8'h66 ? in0 : cpu_a[7:0] == 8'h76 ? in1 :
                  cpu_a[7:0] == 8'h7A ? in2 : 8'h00;
    else if (v_clay)
        port_in = cpu_a[2:0] == 3'd1 ? {in1[7:2], gun_imp != 2'd0, gun_pos != 16'd0} :
                  cpu_a[2:0] == 3'd2 ? gun_pos[7:0] :
                  cpu_a[2:0] == 3'd3 ? shift_q :
                  cpu_a[2:0] == 3'd6 ? gun_pos[15:8] : 8'h00;
    else if (v_shut)                                              // FE DSW, FF inputs (P2 controls on the flip)
        port_in = cpu_a[1:0] == 2'd2 ? in2 :
                  cpu_a[1:0] == 2'd3 ? (sh_flip & cocktail ? (in3 & 8'h3B) | in1 : in3) : 8'h00;
    else case (v_pol ? cpu_a[1:0] : io_a[1:0])                    // polaris: io_a renumbers its writes only
        2'd0:    port_in = in0;
        2'd1:    port_in = in1;
        2'd2:    port_in = in2;
        default: port_in = variant == 8'd0 || variant == 8'd3 || v_afc || v_vtx || v_imul || v_pol || v_ind || v_scv ?
                                                                                          shift_q :   // MB14241 shifter
                           variant == 8'd2 || v_scom ? in3 :   // invasion / spacecom: fourth input port
                                             8'h00;        // no shifter: open (MAME unmapped)
    endcase
end

// work RAM (darthvdr): static RAM, no wait states
logic [7:0] wram_q;
dpram_dc #(.widthad_a(11)) u_wram
(
    .clock_a(clk),
    .address_a(cpu_a[10:0]),
    .data_a(cpu_do),
    .wren_a(ce & ~cpu_wr_n & ~status[4] & is_wram),
    .q_a(wram_q),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b(11'd0),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(),
    .byteena_b(1'b1)
);

// invmulti: memory writes to E000 select the ROM bank {D6, D4, D0}; writes to 6000 drive the EEPROM
// (D0 DI, D4 SK, D6 CS), reads there return its DO on D0
wire  mem_strobe = ~cpu_wr_n & ~status[4];
logic mem_d = 1'b0;
wire  mem_wr = ce & mem_strobe & ~mem_d;
always_ff @(posedge clk) if (ce) mem_d <= mem_strobe;

always_ff @(posedge clk) begin
    if (reset)
        bank <= 3'd0;                       // MAME keeps the bank across a watchdog reset
    else if (mem_wr && is_bank)
        bank <= {cpu_do[6], cpu_do[4], cpu_do[0]};
end

logic eep_do;
eeprom_93c46 u_eeprom
(
    .clk(clk),
    .reset(~rst_n),
    .clear(ioctl_wr0 && ioctl_addr == 25'd0),
    .wr(mem_wr & is_eep),
    .di(cpu_do[0]),
    .sk(cpu_do[4]),
    .cs(cpu_do[6]),
    .dout(eep_do)
);

// data in: INTA -> RST vector, INP -> ports, video RAM -> RAM register, work RAM, EEPROM, else ROM
always_comb begin
    if (status[0])      cpu_di = v_dvdr ? 8'hFF : {3'b110, cnt_e7[2], ~cnt_e7[2], 3'b111};
    else if (status[6]) cpu_di = port_in;
    else if (is_ram)    cpu_di = rr[7:0];
    else if (is_wram)   cpu_di = wram_q;
    else if (is_eep)    cpu_di = {7'd0, eep_do};
    else if (is_cram)   cpu_di = cram_qa;
    else if (is_cram2)  cpu_di = cram2_qa;
    else if (is_star)   cpu_di = 8'h00;
    else if (is_bank)   cpu_di = 8'h00;     // MAME unmapped
    else                cpu_di = rom_q;
end

wire  out_strobe = ~cpu_wr_n & status[4];
logic out_d = 1'b0;
wire  out_wr = ce & out_strobe & ~out_d;

logic [7:0] wd_cnt = 8'd0;
logic [7:0] snd0 = 8'd0;               // extra sound / music latch (port 0, polaris port 2)
logic [7:0] snd3 = 8'd0, snd4 = 8'd0;  // variant 20 sound latches 3 / 4
logic       v128_d = 1'b0;
logic       dv_flip = 1'b0;
logic [1:0] dv_step = 2'd0, dv_play = 2'd0;

always_ff @(posedge clk) begin
    if (!rst_n) begin
        shift_data <= 16'd0;
        shift_cnt  <= 3'd0;
        snd1       <= 8'd0;
        snd2       <= 8'd0;
        snd0       <= 8'd0;
        snd3       <= 8'd0;
        snd4       <= 8'd0;
        rev_sh     <= 1'b0;
        dv_flip    <= 1'b0;
        sh_flip    <= 1'b0;
        dv_step    <= 2'd0;
        dv_play    <= 2'd0;
        out_d      <= 1'b0;
    end else begin
        if (ce) out_d <= out_strobe;
        if (out_wr && v_dvdr) begin
            case (cpu_a[3:0])
                4'd0: dv_flip <= cpu_do[0];
                4'd8: begin
                    snd1 <= cpu_do;
                    if (cpu_do[3] && !snd1[3]) begin                  // fleet: each pulse plays the next of 4 steps
                        dv_play <= dv_step;
                        dv_step <= dv_step + 2'd1;
                    end
                end
                default: ;
            endcase
        end
        else if (out_wr && v_tab) begin
            if (tab_wr[0] | tab_wr[7]) shift_cnt <= cpu_do[2:0];
            if (tab_wr[7])             rev_sh    <= cpu_do[3];
            if (tab_wr[1])             shift_data <= {cpu_do, shift_data[15:8]};
            if (tab_wr[3])             snd1 <= cpu_do;
            if (tab_wr[4])             snd2 <= cpu_do;
            if (tab_wr[5])             snd3 <= cpu_do;
            if (tab_wr[6])             snd4 <= cpu_do;
        end
        else if (out_wr && v_afc) begin
            case (cpu_a[2:0])
                3'd3: shift_data <= {cpu_do, shift_data[15:8]};
                3'd7: shift_cnt  <= cpu_do[2:0];
                default: ;
            endcase
        end
        else if (out_wr && v_shut) begin
            case (cpu_a[1:0])
                2'd1: snd1 <= cpu_do;
                2'd2: snd2 <= cpu_do;
                2'd3: sh_flip <= cpu_do[2];
                default: ;
            endcase
        end
        else if (out_wr && v_scom) begin
            if (cpu_a[1]) snd1 <= cpu_do;
            if (cpu_a[2]) snd2 <= cpu_do;
        end
        else if (out_wr && v_clay) begin
            case (io_a)
                3'd2: shift_cnt  <= cpu_do[2:0];
                3'd4: shift_data <= {cpu_do, shift_data[15:8]};
                default: ;
            endcase
        end
        else if (out_wr) begin
            case (io_a)
                3'd0: snd0       <= cpu_do;
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
        if (out_wr && (v_tab ? tab_wr[2] : io_a == 3'd6))
            wd_cnt <= 8'd0;
        if (wd_cnt >= wd_len && wd_en) begin
            wd_reset <= 1'b1;
            wd_cnt   <= 8'd0;
        end
    end
end

// ---------------------------------------------------------------- video RAM (2000-3FFF, mirror 6000)

// Flip (cocktail / CRT Flip): the bitmap is scanned from the other corner so the picture matches MAME's
// flipped 260 x 224 image. Bytes are fetched one slot early and loaded 4 pixels earlier, bit-reversed.
wire flip = crt_flip ^ (cocktail & (v_dvdr ? dv_flip : v_shut ? sh_flip : snd2[5]));

wire [4:0] col  = {cnt_e5[3:0], cnt_d5[3]};
wire [7:0] row  = v_shut ? (flip ? 8'd223 - vpos : vpos - 8'd32) :   // shuttlei: 192 rows from RAM 0
                  flip ? 8'd31 - vpos : vpos;
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

// vortex colour: bit 0 of the byte after the one fetched, from a copy of the video RAM read in parallel
logic [7:0] sram_q;
dpram_dc #(.widthad_a(13)) u_sram
(
    .clock_a(clk),
    .address_a(ram_a),
    .data_a(cpu_do),
    .wren_a(ram_we & v_vtx),
    .q_a(),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b({row, vcol} + 13'd1),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(sram_q),
    .byteena_b(1'b1)
);

logic [1:0] cdb = 2'd0;             // {next byte bit 0, source column bit 2}, timed as rdb
always_ff @(posedge clk) if (ce) cdb <= {sram_q[0], vcol[2]};

// ---------------------------------------------------------------- family 3 colour (MAME 8080bw_v.cpp)

// colour PROM (2 x 1K, P1 / P2 halves) and polaris cloud graphics: MRA index 0 at 0x20000 / 0x20800. Port A loads
// them while the board is in reset and reads the cloud pattern at run time.
logic [7:0] cprom_q, cloud_q;
logic [7:0] cloud_a;
logic [10:0] p2_a;
wire        cprom_dl = ioctl_wr0 && ioctl_addr[24:12] == 13'h0020;
wire  [1:0] cmap_sel = cflags[7:6];
wire        cmap = cmap_sel == 2'd1 ? snd2[5] : cmap_sel == 2'd2 ? ~snd2[5] : cmap_sel == 2'd3 ? snd2[6] : 1'b0;
wire [10:0] prom_ia = {cmap, row[7:3], vcol};

dpram_dc #(.widthad_a(12)) u_cprom
(
    .clock_a(clk),
    .address_a(reset ? ioctl_addr[11:0] : col_mode == 8'd7 ? {1'b0, p2_a} : {4'b1000, cloud_a}),
    .data_a(ioctl_dout),
    .wren_a(cprom_dl),
    .q_a(cloud_q),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b({1'b0, prom_ia}),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(cprom_q),
    .byteena_b(1'b1)
);

// CPU colour RAM: per 4 rows {A12:7, A4:0} (schaser, polaris, lupin3a, orbite), per 8 rows {A12:8, A4:0}
// (rollingc, second RAM = background colours), cosmo 5C00-5FFF {A9:0}
wire        cram4  = col_mode == 8'd2 || col_mode == 8'd3 || col_mode == 8'd4;
wire [10:0] cram_ca = cram4 ? {cpu_a[12:7], cpu_a[4:0]} : col_mode == 8'd6 ? {1'b0, cpu_a[9:0]} :
                              {1'b0, cpu_a[12:8], cpu_a[4:0]};
wire [10:0] cram_va = cram4 ? {row[7:2], vcol} : {1'b0, row[7:3], vcol};

dpram_dc #(.widthad_a(11)) u_cram
(
    .clock_a(clk),
    .address_a(cram_ca),
    .data_a(cpu_do),
    .wren_a(mem_wr & is_cram),
    .q_a(cram_qa),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b(cram_va),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(cram_qb),
    .byteena_b(1'b1)
);

dpram_dc #(.widthad_a(10)) u_cram2
(
    .clock_a(clk),
    .address_a(cram_ca[9:0]),
    .data_a(cpu_do),
    .wren_a(mem_wr & is_cram2),
    .q_a(cram2_qa),
    .byteena_a(1'b1),

    .clock_b(clk),
    .address_b(cram_va[9:0]),
    .data_b(8'd0),
    .wren_b(1'b0),
    .q_b(cram2_qb),
    .byteena_b(1'b1)
);

// colour of the fetched byte, timed as rdb: {cloud allowed, background pen, foreground pen}
logic [7:0] pq = 8'd0, cq = 8'd0, c2q = 8'd0;
always_ff @(posedge clk) if (ce) begin
    pq  <= cprom_q;
    cq  <= cram_qb;
    c2q <= cram2_qb;
end

wire       red_on = cflags[5] & snd1[2];               // invadpt2 "screen red" (port 3 bit 2)
wire [2:0] cq_f   = cflags[3] ? ~cq[2:0] : cq[2:0];
logic [3:0] g_fore, g_back;
logic       g_cloud;
always_comb begin
    g_fore  = {1'b0, cq_f};
    g_back  = {1'b0, cflags[2:0]};
    g_cloud = 1'b0;
    case (col_mode)
        8'd1: g_fore = red_on ? 4'd1 : {1'b0, pq[2:0]};
        8'd3: g_back = snd2[3] ? 4'd0 : (pq[3:2] == 2'b11 && snd2[4]) ? 4'd4 : 4'd2;   // schaser field control
        8'd4: begin g_fore = {1'b0, ~cq[2:0]}; g_back = pq[0] ? 4'd6 : 4'd2; g_cloud = ~pq[3]; end
        8'd5: begin g_fore = cq[3:0]; g_back = c2q[3:0]; end
        default: ;
    endcase
end
wire [8:0] gdb = {g_cloud, g_back, g_fore};

// ---------------------------------------------------------------- video shift register

logic [7:0] shift = 8'd0;
logic [7:0] hold  = 8'd0;
logic       vid   = 1'b0;
logic [1:0] chold = 2'd0, cshift = 2'd0, cvid = 2'd0;
logic [8:0] ghold = 9'd0, gshift = 9'd0, gvid = 9'd0;

wire v_act     = ~cnt_e7[4];
wire load_norm = ~flip & v_act & ~cnt_e5[4] & (cnt_d5[2:0] == 3'd3);
wire load_flip =  flip & v_act & (cnt_d5[2:0] == 3'd7) & ((~cnt_e5[4] & hx != 9'd255) | hx == 9'd319);

always_ff @(posedge clk) begin
    if (pix) begin
        if (cnt_d5[2:0] == 3'd3) begin
            hold  <= rdb;
            chold <= cdb;
            ghold <= gdb;
        end
        if (load_norm) begin                // shuttlei bytes are MSB first: the bit order swaps
            shift  <= v_shut ? {rdb[0], rdb[1], rdb[2], rdb[3], rdb[4], rdb[5], rdb[6], rdb[7]} : rdb;
            cshift <= cdb;
            gshift <= gdb;
        end
        else if (load_flip) begin
            shift  <= v_shut ? hold : {hold[0], hold[1], hold[2], hold[3], hold[4], hold[5], hold[6], hold[7]};
            cshift <= chold;
            gshift <= ghold;
        end
        else
            shift <= {1'b0, shift[7:1]};
        vid  <= shift[0];
        cvid <= cshift;
        gvid <= gshift;
    end
end

// ---------------------------------------------------------------- sync / blank (MAME framing)

// Registered outputs show one count after their edge: bitmap pixel d is on screen at hx = d + 5, so the
// visible window hx 1-260 = 4 black pixels + 256 bitmap pixels; HSYNC = MAME H 272-287, VSYNC = V 236-239.
// spacecom has no 4-pixel delay: only the 256 bitmap pixels are shown (hx 5-260, flipped hx 1-256).
wire [8:0] hb_end   = (narrow & ~flip) ? 9'd4 : 9'd0;
wire [8:0] hb_start = (narrow &  flip) ? 9'd256 : 9'd260;
always_ff @(posedge clk) begin
    if (pix) begin
        if (hx == hb_end)   video_hblank <= 1'b0;
        if (hx == hb_start) video_hblank <= 1'b1;
        if (hx == 9'd260) begin
            video_hblank <= 1'b1;
            video_vblank <= cnt_e7[4] | (v_shut & vpos >= 8'd224);   // shuttlei: 192 lines
        end
        if (hx == 9'd272) begin
            video_hs <= 1'b1;
            video_vs <= (vcnt >= 9'd486 && vcnt < 9'd490);
        end
        if (hx == 9'd288) video_hs <= 1'b0;
    end
end

// ---------------------------------------------------------------- colour overlay (MAME layout rectangles)

// Rectangles are in picture coordinates (x 0-259 = hx 1-260, rows 0-223), half-open, later ones win. The bands follow
// every flip (CRT Flip and the cocktail flip), so they stay on the score, shields and bases.
logic [7:0] ov_row = 8'd0;
logic [7:0] ov_r = 8'hFF, ov_g = 8'hFF, ov_b = 8'hFF;
logic       xh = 1'b0;
wire  [7:0] row_now = (hx == 9'd0) ? vpos - 8'd32 : ov_row;
wire  [8:0] ox = flip ? 9'd259 - hx : hx;                    // the pixel shown after this edge is x = hx
wire  [7:0] oy = flip ? (v_shut ? 8'd191 : 8'd223) - row_now : row_now;

function [7:0] ovb(input int i);
    ovb = ov_tab[i*8 +: 8];
endfunction

always_ff @(posedge clk) begin
    if (pix) begin : ov_lookup
        logic [7:0] r, g, b, hi;
        logic [8:0] x0, x1;
        if (hx == 9'd0) ov_row <= vpos - 8'd32;
        {r, g, b} = 24'hFFFFFF;
        for (int k = 0; k < 16; k++) begin
            hi = ovb(1 + k*8 + 1);
            x0 = {hi[0], ovb(1 + k*8)};
            x1 = {hi[1], ovb(1 + k*8 + 2)};
            if (k < ovb(0) && ov_en && ox >= x0 && ox < x1 && oy >= ovb(1 + k*8 + 3) && oy < ovb(1 + k*8 + 4))
                {r, g, b} = {ovb(1 + k*8 + 5), ovb(1 + k*8 + 6), ovb(1 + k*8 + 7)};
        end
        ov_r <= r; ov_g <= g; ov_b <= b;
        // crosshair: 9 x 9 cross centred on the gun, picture x = bitmap x + 4
        xh <= xhair_en && v_clay && ((ox == {1'b0, gun_x} + 9'd4 && oy + 8'd4 >= gun_y && oy <= gun_y + 8'd4) ||
                                     (oy == gun_y && ox >= {1'b0, gun_x} && ox <= {1'b0, gun_x} + 9'd8));
    end
end

// polaris clouds: a 16 x 64 pattern drawn on background pixels where the PROM allows it, moving down one line every
// 4 frames (MAME screen_update_polaris / polaris_60hz_w). Bitmap x = picture x - 4, raw V = picture row + 32.
logic [7:0] cloud_pos = 8'd0;
logic [1:0] cloud_div = 2'd0;
logic       cl_vb = 1'b0, cl_bit = 1'b0;
wire  [8:0] cl_x = ox - 9'd4;
wire  [7:0] cl_y = oy + 8'd32 - cloud_pos;
assign      cloud_a = {~cl_y[5:0], cl_x[3:2]};
always_ff @(posedge clk) begin
    if (ce) begin
        cl_vb <= cnt_e7[4];
        if (cnt_e7[4] && !cl_vb) begin
            cloud_div <= cloud_div + 2'd1;
            if (cloud_div == 2'd3) cloud_pos <= cloud_pos + 8'd1;
        end
    end
    if (pix) cl_bit <= col_mode == 8'd4 && cl_y < 8'd64 && cloud_q[{1'b0, ~cl_x[1:0]}];
end

// phantom2 clouds (MAME screen_update_phantom2): row counter = frame start + picture row, 0xE0B-0xFFF, advanced 262
// rows per frame from 0; a pixel x uses PROM byte {counter[7:1], ((x - 16) >> 4)}, bit ((x - 16) >> 1) & 7; pixels 0-15 still
// show the previous row's last byte. Cloud pixels are grey on background pixels.
logic [11:0] p2_frame = 12'd0;         // MAME machine_start: 0, then wraps 0x1000 -> 0xE0B
logic        p2_vb = 1'b0, p2_bit = 1'b0;
wire  [12:0] p2_next = {1'b0, p2_frame} + 13'd262;
wire  [8:0]  p2_x  = ox - 9'd16;
wire  [12:0] p2_r0 = {1'b0, p2_frame} + {5'd0, oy} - (ox < 9'd16 ? 13'd1 : 13'd0);
wire  [12:0] p2_row = p2_r0 >= 13'h1000 ? p2_r0 - 13'h1F5 : p2_r0;
assign       p2_a = {p2_row[7:1], p2_x[7:4]};
always_ff @(posedge clk) begin
    if (ce) begin
        p2_vb <= cnt_e7[4];
        if (cnt_e7[4] && !p2_vb)
            p2_frame <= p2_next >= 13'h1000 ? 12'(p2_next - 13'h1F5) : p2_next[11:0];
    end
    if (pix) p2_bit <= col_mode == 8'd7 && cloud_q[p2_x[3:1]];
end

// pens -> RGB: RBG_3BIT (b0 R, b1 B, b2 G), RGB_3BIT (b0 R, b1 G, b2 B), rollingc 16 pens (b3 intensity, pens 5 / 6
// pink / orange as MAME)
wire [3:0] pen = vid ? gvid[3:0] : (gvid[8] && cl_bit) ? 4'd7 : gvid[7:4];
logic [23:0] pen_rgb;
always_comb begin
    if (col_mode == 8'd5)
        pen_rgb = pen == 4'd5 ? 24'hFF0080 : pen == 4'd6 ? 24'hFF8000 :
                  {{8{pen[2]}}, {8{pen[1]}}, {8{pen[0]}}} & (pen[3] ? 24'hFFFFFF : 24'h7F7F7F);
    else if (cflags[4])
        pen_rgb = {{8{pen[0]}}, {8{pen[1]}}, {8{pen[2]}}};
    else
        pen_rgb = {{8{pen[0]}}, {8{pen[2]}}, {8{pen[1]}}};
end

// vortex: red / green from the next byte's bit 0, blue on source columns 4-7 of every 8 (MAME screen_update_vortex)
wire [7:0] bg = p2_bit ? 8'hC0 : 8'h00;          // phantom2 cloud grey (through the overlay like lit pixels)
assign video_r = xh ? 8'hFF : colour ? pen_rgb[23:16] : !vid ? (p2_bit ? ov_r & 8'hC0 : 8'd0) : v_vtx ? {8{~cvid[1]}} : ov_r;
assign video_g = xh ? 8'h00 : colour ? pen_rgb[15:8]  : !vid ? (p2_bit ? ov_g & bg : 8'd0)    : v_vtx ? {8{ cvid[1]}} : ov_g;
assign video_b = xh ? 8'h00 : colour ? pen_rgb[7:0]   : !vid ? (p2_bit ? ov_b & bg : 8'd0)    : v_vtx ? {8{ cvid[0]}} : ov_b;

// ---------------------------------------------------------------- sound board (port 3 / port 5, 16V = "480 Hz")

// The sample-driven bootlegs' sound bits (MAME 8080bw_a.cpp handlers) remapped onto the board's inputs:
// p1 [0] saucer [1] missile [2] explosion [3] invader hit [4] bonus [5] sound on; p2 [3:0] fleet [4] saucer hit
logic [7:0] sp1, sp2;
always_comb begin
    case (variant)
        8'd3:    begin sp1 = {2'b00, 1'b1, 1'b0, snd1[3:0]}; sp2 = snd2; end
        8'd4:    begin sp1 = {2'b00, snd1[5:1], snd2[3]};
                       sp2 = {3'b000, snd2[2] | snd2[4], snd2[0], 2'b00, snd1[0]}; end
        8'd5:    begin sp1 = {2'b00, snd1[0], snd1[4], snd1[7], snd1[6], snd1[1], snd1[5]};
                       sp2 = {3'b000, snd1[2], snd1[3] ? 4'b0001 << dv_play : 4'b0000}; end
        8'd6, 8'd11: begin sp1 = 8'd0; sp2 = 8'd0; end                 // attackfc / claybust: no sound in MAME, silent
        8'd10:   begin sp1 = {3'b001, snd1[1], sh_pulse[1], sh_pulse[2], sh_pulse[3], snd1[2]};  // FD [0] fleet [1] extra tank [2] UFO,
                       sp2 = {3'b000, sh_pulse[0], 3'b000, snd1[0]}; end  // FE codes as pulses
        default: begin sp1 = snd1; sp2 = snd2; end
    endcase
    // family 3: MAME's sample handlers mapped onto the board's voices (invaders samples: 0 missile, 1 explosion,
    // 2 invader hit, 3 saucer hit, 4-7 fleet, 8 bonus)
    case (smap)
        8'd1, 8'd7: begin sp1 = 8'd0; sp2 = 8'd0; end                              // silent / lrescue (tune only)
        8'd2: begin sp1 = {2'b00, snd1[5], snd1[4], snd1[0] | snd2[4], snd1[2], snd1[1], 1'b0};     // ballbomb
                    sp2 = {4'b0000, snd1[3] | snd2[0], 2'b00, snd2[2]}; end
        8'd3: begin sp1 = {2'b00, snd1[5], 1'b0, snd1[3] | snd2[1], snd1[1], snd2[0] | snd2[3], 1'b0};  // indianbt
                    sp2 = {3'b000, snd1[2], snd1[0] | snd2[4], 3'b000}; end
        8'd4: begin sp1 = {2'b00, snd1[5], 1'b0, snd1[3], snd1[2], 2'b00};                    // indianbtbr
                    sp2 = {3'b000, snd2[3], snd1[0] | snd2[4], 3'b000}; end
        8'd5: begin sp1 = {2'b00, snd2[4], 2'b00, snd1[4], 2'b00};                            // schasercv
                    sp2 = {5'b00000, snd1[1], 2'b00}; end
        8'd6: begin sp1 = snd1 | {3'b000, snd0[4], 1'b0, snd0[2], snd0[1], 1'b0}; sp2 = snd2; end   // rollingc port 0
        8'd8: begin sp1 = {2'b00, 1'b1, 1'b0, snd1[3:0]}; sp2 = snd2; end                     // spcewarla
        default: ;
    endcase
end

// shuttlei FE sound codes (MAME sh_port_2_w): 23 hit, 2B shoot, A3 UFO hit, AB death -> ~10 ms trigger pulses
// sh_pulse: [0] UFO hit, [1] invader hit, [2] explosion, [3] missile
logic [3:0]  sh_pulse = 4'd0;
logic [16:0] sh_ptim = 17'd0;
always_ff @(posedge clk) begin
    if (!rst_n) begin
        sh_pulse <= 4'd0;
        sh_ptim  <= 17'd0;
    end else begin
        if (ce && sh_ptim != 17'd0) sh_ptim <= sh_ptim - 17'd1;
        if (ce && sh_ptim == 17'd1) sh_pulse <= 4'd0;
        if (out_wr && v_shut && cpu_a[1:0] == 2'd2) begin
            case (cpu_do)
                8'h23: sh_pulse <= 4'b0010;
                8'h2B: sh_pulse <= 4'b1000;
                8'hA3: sh_pulse <= 4'b0001;
                8'hAB: sh_pulse <= 4'b0100;
                default: ;
            endcase
            if (cpu_do == 8'h23 || cpu_do == 8'h2B || cpu_do == 8'hA3 || cpu_do == 8'hAB) sh_ptim <= 17'd99840;
        end
    end
end

logic signed [15:0] board_snd;
invaders_sound u_sound
(
    .clk(clk),
    .reset(~rst_n),
    .pause(pause),
    .p1(sp1),
    .p2(sp2),
    .v16(cnt_e7[0]),
    .taito(taito_snd),
    .out(board_snd)
);

// invad2ct: the second player's invaders sound board on latches 3 / 4, right channel
logic signed [15:0] board2_snd;
invaders_sound u_sound2
(
    .clk(clk),
    .reset(~rst_n | smap != 8'd9),
    .pause(pause),
    .p1(snd3),
    .p2(snd4),
    .v16(cnt_e7[0]),
    .taito(taito_snd),
    .out(board2_snd)
);
assign audio_r = smap == 8'd9 ? board2_snd : audio;

// 1-bit tune speakers the CPU bit-bangs: spcewars / spcewarla port 3 (4) bit 4, lrescue port 5 bit 3 (sound
// enable port 3 bit 5), schasercv port 5 bit 0
wire tune_bit = (variant == 8'd3 || smap == 8'd8) ? snd1[4] : smap == 8'd7 ? snd2[3] & snd1[5] :
                smap == 8'd5 ? snd2[0] : 1'b0;
wire signed [16:0] tune = tune_bit ? 17'sd6000 : 17'sd0;
wire signed [16:0] mix  = board_snd + tune;
assign audio = mix > 17'sd32767 ? 16'sd32767 : 16'(mix);

endmodule
