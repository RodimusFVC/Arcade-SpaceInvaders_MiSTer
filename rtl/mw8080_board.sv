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
//   9 invmulti (128K scrambled ROM banked into 0000/4000 by writes to E000, RAM 2000 only, 93C46 EEPROM at 6000)
// rom_dec (MRA index 1 byte 3): [0] ROM A8/A9 swapped (attackfc), [1] ROM A0/A3/A9 inverted (vortex)

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
    input  logic  [1:0] rom_dec,

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
wire wd_en   = variant <= 8'd2 || v_vtx || v_imul;                // boards with the port 6 watchdog
wire is_ram  = v_dvdr ? cpu_a[15:13] == 3'b010 :                  // the shared (video) DRAM
               v_imul ? cpu_a[14:13] == 2'b01 : cpu_a[13];
wire is_eep  = v_imul & cpu_a[15:13] == 3'b011;                   // invmulti EEPROM 6000-7FFF
wire is_bank = v_imul & cpu_a[15:13] == 3'b111;                   // invmulti bank latch E000-FFFF (write only)
wire is_wram = v_dvdr & cpu_a[15:11] == 5'b00011;                 // darthvdr work RAM 1800-1FFF

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
logic [7:0] rom_raw;
logic [2:0] bank = 3'd0;
wire [12:0] rom_a = rom_dec[0] ? {cpu_a[12:10], cpu_a[8], cpu_a[9], cpu_a[7:0]} :   // decrypted on the read path
                    rom_dec[1] ? cpu_a[12:0] ^ 13'h0209 :
                                 cpu_a[12:0];
wire [16:0] im_i  = {bank, cpu_a[14], cpu_a[12:0]};
wire [16:0] rom_ra = v_imul ? {im_i[16], im_i[15], im_i[11], im_i[12], im_i[13], im_i[8], im_i[14], im_i[9], im_i[10],
                               im_i[7:0]} :
                              {2'b00, cpu_a[14], 1'b0, rom_a};
wire  [7:0] rom_q = v_imul ? {rom_raw[0], rom_raw[6], rom_raw[5], rom_raw[7], rom_raw[4], rom_raw[3], rom_raw[1], rom_raw[2]} :
                             rom_raw;
wire        rom_dl = ioctl_wr0 && ioctl_addr < 25'h20000;

dpram_dc #(.widthad_a(17)) u_rom
(
    .clock_a(clk),
    .address_a(ioctl_addr[16:0]),
    .data_a(ioctl_dout),
    .wren_a(rom_dl),
    .q_a(),
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

// port number as the invaders map sees it: vortex inverts A1; spacecom decodes ports 41 / 42 / 44 one-hot
wire [2:0] io_a = v_vtx  ? cpu_a[2:0] ^ 3'b010 :
                  v_scom ? (cpu_a[0] ? 3'd1 : cpu_a[1] ? 3'd2 : cpu_a[2] ? 3'd3 : 3'd0) :
                           cpu_a[2:0];

logic [7:0] port_in;
always_comb begin
    case (io_a[1:0])
        2'd0:    port_in = in0;
        2'd1:    port_in = in1;
        2'd2:    port_in = in2;
        default: port_in = variant == 8'd0 || variant == 8'd3 || v_afc || v_vtx || v_imul ? shift_q :   // MB14241 shifter
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
    else if (is_bank)   cpu_di = 8'h00;     // MAME unmapped
    else                cpu_di = rom_q;
end

wire  out_strobe = ~cpu_wr_n & status[4];
logic out_d = 1'b0;
wire  out_wr = ce & out_strobe & ~out_d;

logic [7:0] wd_cnt = 8'd0;
logic       v128_d = 1'b0;
logic       dv_flip = 1'b0;
logic [1:0] dv_step = 2'd0, dv_play = 2'd0;

always_ff @(posedge clk) begin
    if (!rst_n) begin
        shift_data <= 16'd0;
        shift_cnt  <= 3'd0;
        snd1       <= 8'd0;
        snd2       <= 8'd0;
        dv_flip    <= 1'b0;
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
        else if (out_wr && v_afc) begin
            case (cpu_a[2:0])
                3'd3: shift_data <= {cpu_do, shift_data[15:8]};
                3'd7: shift_cnt  <= cpu_do[2:0];
                default: ;
            endcase
        end
        else if (out_wr && v_scom) begin
            if (cpu_a[1]) snd1 <= cpu_do;
            if (cpu_a[2]) snd2 <= cpu_do;
        end
        else if (out_wr) begin
            case (io_a)
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
        if (out_wr && io_a == 3'd6)
            wd_cnt <= 8'd0;
        if (wd_cnt == 8'd255 && wd_en) begin
            wd_reset <= 1'b1;
            wd_cnt   <= 8'd0;
        end
    end
end

// ---------------------------------------------------------------- video RAM (2000-3FFF, mirror 6000)

// Flip (cocktail / CRT Flip): the bitmap is scanned from the other corner so the picture matches MAME's
// flipped 260 x 224 image. Bytes are fetched one slot early and loaded 4 pixels earlier, bit-reversed.
wire flip = crt_flip ^ (cocktail & (v_dvdr ? dv_flip : snd2[5]));

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

// ---------------------------------------------------------------- video shift register

logic [7:0] shift = 8'd0;
logic [7:0] hold  = 8'd0;
logic       vid   = 1'b0;
logic [1:0] chold = 2'd0, cshift = 2'd0, cvid = 2'd0;

wire v_act     = ~cnt_e7[4];
wire load_norm = ~flip & v_act & ~cnt_e5[4] & (cnt_d5[2:0] == 3'd3);
wire load_flip =  flip & v_act & (cnt_d5[2:0] == 3'd7) & ((~cnt_e5[4] & hx != 9'd255) | hx == 9'd319);

always_ff @(posedge clk) begin
    if (pix) begin
        if (cnt_d5[2:0] == 3'd3) begin
            hold  <= rdb;
            chold <= cdb;
        end
        if (load_norm) begin
            shift  <= rdb;
            cshift <= cdb;
        end
        else if (load_flip) begin
            shift  <= {hold[0], hold[1], hold[2], hold[3], hold[4], hold[5], hold[6], hold[7]};
            cshift <= chold;
        end
        else
            shift <= {1'b0, shift[7:1]};
        vid  <= shift[0];
        cvid <= cshift;
    end
end

// ---------------------------------------------------------------- sync / blank (MAME framing)

// Registered outputs show one count after their edge: bitmap pixel d is on screen at hx = d + 5, so the
// visible window hx 1-260 = 4 black pixels + 256 bitmap pixels; HSYNC = MAME H 272-287, VSYNC = V 236-239.
// spacecom has no 4-pixel delay: only the 256 bitmap pixels are shown (hx 5-260, flipped hx 1-256).
wire [8:0] hb_end   = (v_scom & ~flip) ? 9'd4 : 9'd0;
wire [8:0] hb_start = (v_scom &  flip) ? 9'd256 : 9'd260;
always_ff @(posedge clk) begin
    if (pix) begin
        if (hx == hb_end)   video_hblank <= 1'b0;
        if (hx == hb_start) video_hblank <= 1'b1;
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

// ---------------------------------------------------------------- colour overlay (MAME layout rectangles)

// Rectangles are in picture coordinates (x 0-259 = hx 1-260, rows 0-223), half-open, later ones win. The bands follow
// every flip (CRT Flip and the cocktail flip), so they stay on the score, shields and bases.
logic [7:0] ov_row = 8'd0;
logic [7:0] ov_r = 8'hFF, ov_g = 8'hFF, ov_b = 8'hFF;
wire  [7:0] row_now = (hx == 9'd0) ? vpos - 8'd32 : ov_row;
wire  [8:0] ox = flip ? 9'd259 - hx : hx;                    // the pixel shown after this edge is x = hx
wire  [7:0] oy = flip ? 8'd223 - row_now : row_now;

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
    end
end

// vortex: red / green from the next byte's bit 0, blue on source columns 4-7 of every 8 (MAME screen_update_vortex)
assign video_r = !vid ? 8'd0 : v_vtx ? {8{~cvid[1]}} : ov_r;
assign video_g = !vid ? 8'd0 : v_vtx ? {8{ cvid[1]}} : ov_g;
assign video_b = !vid ? 8'd0 : v_vtx ? {8{ cvid[0]}} : ov_b;

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
        8'd6:    begin sp1 = 8'd0; sp2 = 8'd0; end                     // attackfc: sound board unknown, silent
        default: begin sp1 = snd1; sp2 = snd2; end
    endcase
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

// spcewars: port 3 bit 4 drives a speaker directly (the CPU bit-bangs its tunes)
wire signed [16:0] tune = (variant == 8'd3 && snd1[4]) ? 17'sd6000 : 17'sd0;
wire signed [16:0] mix  = board_snd + tune;
assign audio = mix > 17'sd32767 ? 16'sd32767 : 16'(mix);

endmodule
