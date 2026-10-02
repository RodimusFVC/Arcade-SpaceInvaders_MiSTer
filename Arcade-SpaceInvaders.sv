//============================================================================
//
//  Midway / Taito 8080 hardware (Space Invaders) for MiSTer
//  Copyright (C) 2026 Rodimus
//
//  Board model derived from the Space Invaders core by Daniel Wallner (c) 2002
//  and MikeJ, and its MiST / MiSTer port by Gehstock, Gyurco, David Woods,
//  Mike Coates, Shane Lynch and Alan Steremberg
//
//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//
//============================================================================

module emu
(
    `include "sys/emu_ports.vh"
);

wire        CLK_40M;
wire        locked;
wire [127:0] status;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire [10:0] ps2_key;
wire        ioctl_download;
wire        ioctl_upload;
wire        ioctl_upload_req;
wire  [7:0] ioctl_din;
wire        ioctl_wr;
wire  [7:0] ioctl_index;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire [15:0] joystick_0, joystick_1, joystick_2, joystick_3;
wire [15:0] joy_la0, joy_la1;   // analog sticks {Y, X}, signed, -Y = up
wire [24:0] ps2_mouse;
wire  [7:0] paddle_0, paddle_1;
wire  [8:0] spinner_0, spinner_1;
wire [21:0] gamma_bus;
wire        direct_video;
wire        video_rotated;
wire        pause_cpu;
wire        hblank, vblank;

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {SDRAM_DQ, SDRAM_A, SDRAM_BA, SDRAM_CLK, SDRAM_CKE, SDRAM_DQML, SDRAM_DQMH, SDRAM_nWE, SDRAM_nCAS, SDRAM_nRAS, SDRAM_nCS} = 'Z;

assign VGA_F1 = 0;
assign VGA_SCALER = 0;
assign VGA_DISABLE = 0;
assign FB_FORCE_BLANK = 0;
assign HDMI_FREEZE = 0;
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

wire signed [15:0] audio, audio_r;     // audio_r: second sound board (invad2ct), else the same as audio
wire signed [15:0] b8_audio, b8_audio_r;
wire zac;                                   // board select (MRA variant 32 / 33): Zaccaria 1B1120
wire signed [15:0] z_audio;
wire signed [15:0] ds_audio;
wire               ds_on;
assign audio   = zac ? z_audio : ds_on ? ds_audio : b8_audio;
assign audio_r = zac ? audio : ds_on ? ds_audio : b8_audio_r;
assign AUDIO_L = pause_cpu ? 16'd0 : audio;
assign AUDIO_R = pause_cpu ? 16'd0 : audio_r;
assign AUDIO_S = 1;   // signed
assign AUDIO_MIX = 0;

assign LED_DISK  = 0;
assign LED_POWER = 0;
assign LED_USER  = ioctl_download;
assign BUTTONS = 0;

///////////////////////////////////////////////////

// MRA index 1:
//   byte 0      board variant (see rtl/mw8080_board.sv)
//   byte 1      flags: [4] vertical, [7] vertical is ROT90
//   byte 2      sound board: [0] Taito L-shaped (else Midway)
//   byte 3      ROM decode: [0] A8/A9 swapped, [1] A0/A3/A9 inverted, [2] nibble bproms (rtl/mw8080_board.sv)
//   bytes 4-11  DIP switch bytes 0-7 at their MRA defaults (MiSTer sends index 254 only for MRAs with a <dip>)
//   bytes 12-15 colour mode, colour flags, sound map, board flags (rtl/mw8080_board.sv)
//   bytes 48-63 family 4 port tables: read source / write targets per port A2-A0 (rtl/mw8080_board.sv)
//   bytes 200-296 analog fields: 4 descriptors (8 bytes), 64 bytes of remap tables, select line; gen_mra analog_bytes
//   bytes 64+   colour overlay: count, then 8 bytes per rectangle (rtl/mw8080_board.sv)
//   bytes 16-47 input map, one byte per port bit (IN0, IN1, IN2, IN3; bit 0 first): control id, 0 = none
// DIP switch bytes 0-3 hold the idle level of every bit of IN0-IN3; a pressed control inverts its bit
reg [7:0] game_var   = 8'd0;
reg [7:0] game_flags = 8'h10;
reg [7:0] snd_flags  = 8'd0;
reg [7:0] rom_dec    = 8'd0;
reg [31:0] f3_cfg    = 32'd0;
reg [6:0] in_map[32];
reg [127:0] io_tab = 128'd0;
reg  [7:0] an_cfg[97];              // analog descriptors (0-31), remap tables (32-95), select line (96)
reg [1031:0] ov_tab = 1032'd0;

always @(posedge CLK_40M) begin
    if (ioctl_wr && ioctl_index == 8'd1) begin
        if (ioctl_addr == 25'd0) game_var   <= ioctl_dout;
        if (ioctl_addr == 25'd1) game_flags <= ioctl_dout;
        if (ioctl_addr == 25'd2) snd_flags  <= ioctl_dout;
        if (ioctl_addr == 25'd3) rom_dec    <= ioctl_dout;
        if (ioctl_addr >= 25'd12 && ioctl_addr < 25'd16) f3_cfg[ioctl_addr[1:0]*8 +: 8] <= ioctl_dout;
        if (ioctl_addr[24:5] == 20'd0 && ioctl_addr[4]) in_map[{1'b0, ioctl_addr[3:0]}] <= ioctl_dout[6:0];
        if (ioctl_addr[24:5] == 20'd1 && ioctl_addr[4] == 1'b0) in_map[{1'b1, ioctl_addr[3:0]}] <= ioctl_dout[6:0];
        if (ioctl_addr >= 25'd48 && ioctl_addr < 25'd64) io_tab[ioctl_addr[3:0]*8 +: 8] <= ioctl_dout;
        if (ioctl_addr == 25'd0) begin                                                  // a set without analog fields
            for (int i = 0; i < 4; i++) an_cfg[i*8] <= 8'd0;
            an_cfg[96] <= 8'd0;
        end
        if (ioctl_addr >= 25'd200 && ioctl_addr < 25'd297) an_cfg[7'(ioctl_addr - 25'd200)] <= ioctl_dout;
        if (ioctl_addr == 25'd0) ov_tab[7:0] <= 8'd0;                                   // an MRA without overlay data
        if (ioctl_addr >= 25'd64 && ioctl_addr < 25'd193) ov_tab[(ioctl_addr - 25'd64) * 8 +: 8] <= ioctl_dout;
    end
end

assign zac = game_var >= 8'd32;
wire game_vert = game_flags[4];
wire vert_view = game_vert & ~status[12];

wire [1:0] ar = status[9:8];

assign VIDEO_ARX = (!ar) ? (vert_view ? 12'd3 : 12'd4) : (ar - 1'd1);
assign VIDEO_ARY = (!ar) ? (vert_view ? 12'd4 : 12'd3) : 12'd0;

// Status bits: the old core was "A.INVADERS", so none of its saved settings load here
`include "build_id.v"
localparam CONF_STR = {
	"SPACEINV;;",
	"P1,Video Options;",
	"P1O89,Aspect Ratio,Original,Full screen,[ARC1],[ARC2];",
	"P1OC,Orientation,Vert,Horz;",
	"P1OB,HDMI Flip,Off,On;",
	"P1OM,CRT Flip,Off,On;",
	"P1OGI,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"P2,Game Options;",
	"P2ON,Overlay,On,Off;",
	"P2OO,Crosshair,On,Off;",
	"P2OPQ,Trackball Speed,Normal,Fast,Slow;",
	"-;",
	"P3,Pause Options;",
	"P3OJ,Pause when OSD is open,On,Off;",
	"P3OK,Dim video after 10s,On,Off;",
	"-;",
	"P4,High Score Options;",
	"P4OL,Autosave Hiscores,Off,On;",
	"-;",
	"DIP;",
	"-;",
	"R0,Reset;",
	"J1,Fire,Btn 2,Btn 3,Btn 4,Coin,Start 1P,Start 2P,Pause,Btn 5,Btn 6;",
	"jn,A,Y,B,X,Select,Start,R,L;",
	"V,v",`BUILD_DATE
};

hps_io #(.CONF_STR(CONF_STR)) hps_io
(
	.clk_sys(CLK_40M),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.direct_video(direct_video),
	.video_rotated(video_rotated),

	.forced_scandoubler(forced_scandoubler),

	.buttons(buttons),
	.status(status),
	.status_menumask({direct_video}),

	.ioctl_download(ioctl_download),
	.ioctl_upload(ioctl_upload),
	.ioctl_upload_req(ioctl_upload_req),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_din(ioctl_din),
	.ioctl_index(ioctl_index),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.joystick_2(joystick_2),
	.joystick_3(joystick_3),
	.joystick_l_analog_0(joy_la0),
	.joystick_l_analog_1(joy_la1),
	.ps2_mouse(ps2_mouse),
	.paddle_0(paddle_0),
	.paddle_1(paddle_1),
	.spinner_0(spinner_0),
	.spinner_1(spinner_1),
	.ps2_key(ps2_key)
);

////////////////////   CLOCKS   ///////////////////

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(CLK_40M),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll),
	.locked(locked)
);

wire [63:0] reconfig_to_pll;
wire [63:0] reconfig_from_pll;
wire        cfg_waitrequest;

pll_cfg pll_cfg
(
	.mgmt_clk(CLK_50M),
	.mgmt_reset(0),
	.mgmt_waitrequest(cfg_waitrequest),
	.mgmt_read(0),
	.mgmt_readdata(),
	.mgmt_write(0),
	.mgmt_address(0),
	.mgmt_writedata(0),
	.reconfig_to_pll(reconfig_to_pll),
	.reconfig_from_pll(reconfig_from_pll)
);

// Hold the CPU in reset until the PLL is locked and the ROM download has finished
wire reset = RESET | status[0] | buttons[1] | ioctl_download | ~locked;

///////////////////         Keyboard           //////////////////

reg kb_up = 0, kb_down = 0, kb_left = 0, kb_right = 0;
reg kb_b1 = 0, kb_b2 = 0;
reg kb_coin1 = 0, kb_coin2 = 0, kb_start1 = 0, kb_start2 = 0, kb_pause = 0, kb_service = 0, kb_tilt = 0;

wire       pressed = ~ps2_key[9];
wire [7:0] code    = ps2_key[7:0];

always @(posedge CLK_40M) begin
	reg old_state;
	old_state <= ps2_key[10];
	if (old_state != ps2_key[10]) begin
		case (code)
			'h16: kb_start1  <= pressed; // 1
			'h1E: kb_start2  <= pressed; // 2
			'h2E: kb_coin1   <= pressed; // 5
			'h36: kb_coin2   <= pressed; // 6
			'h46: kb_service <= pressed; // 9
			'h2C: kb_tilt    <= pressed; // T
			'h4D: kb_pause   <= pressed; // P

			'h75: kb_up      <= pressed; // up
			'h72: kb_down    <= pressed; // down
			'h6B: kb_left    <= pressed; // left
			'h74: kb_right   <= pressed; // right
			'h14: kb_b1      <= pressed; // ctrl
			'h11: kb_b2      <= pressed; // alt
		endcase
	end
end

//////////////////  Arcade Buttons/Interfaces   ///////////////////////////

// Joystick bits: 0 R, 1 L, 2 D, 3 U, 4-7 Btn 1-4, 8 Coin, 9 Start 1P, 10 Start 2P, 11 Pause, 12-13 Btn 5-6
// analog sticks as 4 directions {U, D, L, R} (threshold +/-40 of 127)
function [3:0] stick(input [15:0] a);
    stick = {$signed(a[15:8]) < -8'sd40, $signed(a[15:8]) > 8'sd40, $signed(a[7:0]) < -8'sd40, $signed(a[7:0]) > 8'sd40};
endfunction
wire [3:0] al1 = stick(joy_la0), al2 = stick(joy_la1);

wire [3:0] dir1 = {joystick_0[3] | kb_up, joystick_0[2] | kb_down, joystick_0[1] | kb_left, joystick_0[0] | kb_right} |
                  al1;
wire [3:0] dir2 = joystick_1[3:0] | al2;   // {U, D, L, R}

wire m_pause = joystick_0[11] | kb_pause;

// DIP switches arrive from the OSD via ioctl index 254
reg [7:0] dip_sw[8] = '{8'hFF,8'hFF,8'hFF,8'hFF,8'h00,8'h00,8'h00,8'h00};
always @(posedge CLK_40M) begin
	if (ioctl_wr && (ioctl_index == 8'd254) && !ioctl_addr[24:3])
		dip_sw[ioctl_addr[2:0]] <= ioctl_dout;
	if (ioctl_wr && (ioctl_index == 8'd1) && ioctl_addr >= 25'd4 && ioctl_addr < 25'd12)
		dip_sw[ioctl_addr[2:0] - 3'd4] <= ioctl_dout;
end

wire       an_sel;                  // the board's controller-select line (clowns / spacwalk / desertgu)
wire [3:0] dsel = an_sel ? dip_sw[4][3:0] : dip_sw[3][3:0];

// control ids used by the MRA input map
wire [127:0] ctl =
{
	52'd0,
	joystick_3[9] | joystick_3[10],                 // 75 start 4: player 4's start button
	joystick_2[9] | joystick_2[10],                 // 74 start 3: player 3's start button
	joystick_3[4],                                  // 73 P4 Btn 1
	joystick_3[0], joystick_3[1], joystick_3[2], joystick_3[3],   // 72 R, 71 L, 70 D, 69 U
	joystick_2[4],                                  // 68 P3 Btn 1
	joystick_2[0], joystick_2[1], joystick_2[2], joystick_2[3],   // 67 R, 66 L, 65 D, 64 U
	4'd0,                                           // 63-60 unused
	dsel,                                           // 59-56 DIP byte 4 / 3 bits 3-0 by the select line
	dip_sw[4],                                      // 55-48 DIP byte 4 (MAME fake port lines)
	dip_sw[3],                                      // 47-40 DIP byte 3 (MAME fake port lines)
	3'd0,
	vblank,                                         // 36 VBLANK
	joystick_1[13:12],                              // 35-34 P2 Btn 6-5
	joystick_0[13:12],                              // 33-32 P1 Btn 6-5
	8'd0,                                           // 31-24 unused
	1'b0,                                           // 23 coin 3
	kb_service,                                     // 22 service
	kb_tilt,                                        // 21 tilt
	joystick_0[10] | joystick_1[10] | kb_start2,    // 20 start 2
	joystick_0[9]  | joystick_1[9]  | kb_start1,    // 19 start 1
	joystick_1[8] | kb_coin2,                       // 18 coin 2
	joystick_0[8] | kb_coin1,                       // 17 coin 1
	joystick_1[7:4],                                // 16-13 P2 Btn 4-1
	dir2[0], dir2[1], dir2[2], dir2[3],             // 12 R, 11 L, 10 D, 9 U
	joystick_0[7], joystick_0[6], joystick_0[5] | kb_b2, joystick_0[4] | kb_b1 | ps2_mouse[0],   // 8-5 P1 Btn 4-1
	dir1[0], dir1[1], dir1[2], dir1[3],             // 4 R, 3 L, 2 D, 1 U
	1'b0                                            // 0 none
};

// Analog controls (Midway one-offs): paddles / wheels / periscope from the MiSTer paddle, spinner, mouse X, left stick
// or D-pad; gun angles from Btn 2 / 3; pedal from Btn 1 or the stick pushed up. Each MRA descriptor turns a channel
// into a port field: linear between min / max (optionally reversed) or N positions through a MAME remap table.
reg  [7:0] pad[2]  = '{8'd128, 8'd128};
reg  [7:0] aim[2]  = '{8'd128, 8'd128};
reg  [7:0] pady    = 8'd255;         // P1 vertical (spcenctr thrust: MAME default position 19 of 20)
reg  [7:0] tb[2]   = '{8'd0, 8'd0};  // trackball X / Y counters (wrap)
reg        an_vbl = 1'b0, an_mtog = 1'b0, sp0_d = 1'b0, sp1_d = 1'b0;
reg  [7:0] pd0_d = 8'd0, pd1_d = 8'd0;

function automatic [7:0] padd(input [7:0] v, input signed [9:0] d);
	reg signed [10:0] t;
	t = $signed({3'b000, v}) + d;
	padd = t < 0 ? 8'd0 : t > 255 ? 8'd255 : t[7:0];
endfunction

wire signed [7:0] an_x0 = joy_la0[7:0], an_x1 = joy_la1[7:0], an_y0 = joy_la0[15:8];
wire signed [9:0] an_v0 = (an_x0 > 8'sd12 || an_x0 < -8'sd12) ? (10'(an_x0) >>> 4) : dir1[0] ? 10'sd3 : dir1[1] ? -10'sd3 : 10'sd0;
wire signed [9:0] an_v1 = (an_x1 > 8'sd12 || an_x1 < -8'sd12) ? (10'(an_x1) >>> 4) : dir2[0] ? 10'sd3 : dir2[1] ? -10'sd3 : 10'sd0;
wire signed [9:0] an_vy = (an_y0 > 8'sd12 || an_y0 < -8'sd12) ? (10'(an_y0) >>> 4) : dir1[2] ? 10'sd3 : dir1[3] ? -10'sd3 : 10'sd0;
wire signed [9:0] an_mx = {ps2_mouse[4], ps2_mouse[4], ps2_mouse[15:8]};
wire signed [9:0] an_my = -{ps2_mouse[5], ps2_mouse[5], ps2_mouse[23:16]};   // PS/2 Y is positive upward
wire signed [9:0] an_sx = 10'($signed(spinner_0[7:0]));
wire signed [9:0] tb_sx = (an_x0 > 8'sd12 || an_x0 < -8'sd12) ? (10'(an_x0) >>> 3) : dir1[0] ? 10'sd8 : dir1[1] ? -10'sd8 : 10'sd0;
wire signed [9:0] tb_sy = (an_y0 > 8'sd12 || an_y0 < -8'sd12) ? (10'(an_y0) >>> 3) : dir1[2] ? 10'sd8 : dir1[3] ? -10'sd8 : 10'sd0;
wire       [7:0] gx, gy;            // light gun position scaled to 0-255 (assigned with the gun below)

// trackball speed (Game Options): Normal 1/2, Fast 1/1, Slow 1/4 of the mouse / stick delta
function automatic [7:0] tbd(input [7:0] v, input signed [9:0] d, input [1:0] spd);
	tbd = v + 8'(spd == 2'd1 ? d : spd == 2'd2 ? d >>> 2 : d >>> 1);
endfunction

wire       [7:0] pedal = (joystick_0[4] | kb_b1) ? 8'd255 : (an_y0 == -8'sd128) ? 8'd255 : (an_y0 < -8'sd12) ? 8'(-{an_y0, 1'b0}) : 8'd0;

always @(posedge CLK_40M) begin
	an_vbl  <= vblank;
	an_mtog <= ps2_mouse[24];
	sp0_d   <= spinner_0[8];
	sp1_d   <= spinner_1[8];
	pd0_d   <= paddle_0;
	pd1_d   <= paddle_1;
	if (paddle_0 != pd0_d)                pad[0] <= paddle_0;
	else if (spinner_0[8] != sp0_d)       pad[0] <= padd(pad[0], 10'($signed(spinner_0[7:0])));
	else if (ps2_mouse[24] != an_mtog)    pad[0] <= padd(pad[0], {ps2_mouse[4], ps2_mouse[4], ps2_mouse[15:8]});
	else if (vblank && !an_vbl)           pad[0] <= padd(pad[0], an_v0);
	if (paddle_1 != pd1_d)                pad[1] <= paddle_1;
	else if (spinner_1[8] != sp1_d)       pad[1] <= padd(pad[1], 10'($signed(spinner_1[7:0])));
	else if (vblank && !an_vbl)           pad[1] <= padd(pad[1], an_v1);
	if (ps2_mouse[24] != an_mtog)          pady <= padd(pady, an_my);
	else if (vblank && !an_vbl)            pady <= padd(pady, an_vy);
	if (ps2_mouse[24] != an_mtog) begin
		tb[0] <= tbd(tb[0], an_mx, status[26:25]);
		tb[1] <= tbd(tb[1], an_my, status[26:25]);
	end else if (spinner_0[8] != sp0_d)
		tb[0] <= tbd(tb[0], an_sx, status[26:25]);
	else if (vblank && !an_vbl) begin
		tb[0] <= tbd(tb[0], tb_sx, status[26:25]);
		tb[1] <= tbd(tb[1], tb_sy, status[26:25]);
	end
	if (vblank && !an_vbl) begin
		aim[0] <= padd(aim[0], joystick_0[5] ? 10'sd4 : joystick_0[6] ? -10'sd4 : 10'sd0);
		aim[1] <= padd(aim[1], joystick_1[5] ? 10'sd4 : joystick_1[6] ? -10'sd4 : 10'sd0);
	end
end

reg [7:0] an_mask[4], an_val[4];
always @* begin
	for (int p = 0; p < 4; p++) begin an_mask[p] = 8'd0; an_val[p] = 8'd0; end
	for (int k = 0; k < 4; k++) begin : an_field
		reg [7:0] d0, port, lsb, wid, a, bmax, fl, base, raw, v, fm;
		reg [15:0] prod;
		reg [5:0] pos;
		d0 = an_cfg[k*8]; port = an_cfg[k*8+1]; lsb = an_cfg[k*8+2]; wid = an_cfg[k*8+3];
		a = an_cfg[k*8+4]; bmax = an_cfg[k*8+5]; fl = an_cfg[k*8+6]; base = an_cfg[k*8+7];
		pos = 6'd0; v = 8'd0; prod = 16'd0;
		case (d0[3:0])
			4'd0:  raw = pad[0];
			4'd1:  raw = pad[1];
			4'd2:  raw = aim[0];
			4'd3:  raw = aim[1];
			4'd4:  raw = pedal;
			4'd5:  raw = an_sel ? pad[1] : pad[0];
			4'd6:  raw = pady;
			4'd7:  raw = tb[0];
			4'd8:  raw = tb[1];
			4'd10: raw = gx;
			4'd11: raw = gy;
			default: raw = 8'd0;
		endcase
		if (fl[3]) begin                                    // N positions, optionally through the remap table
			prod = raw * a;
			pos  = fl[0] ? 6'(a - 8'd1 - prod[15:8]) : prod[13:8];
			v    = fl[4] ? an_cfg[32 + 6'(base[5:0] + {fl[2], 5'd0} + pos)] : {2'b00, pos};
		end else begin                                      // linear min .. max
			prod = raw * (bmax - a + 8'd1);
			v = a + prod[15:8];
			if (fl[0]) v = a + bmax - v;
		end
		if (fl[1]) v = ~v;
		fm = 8'hFF >> (4'd8 - wid[3:0]);
		if (d0[7] && !(fl[5] && an_sel) && !(fl[6] && !an_sel)) begin    // [5] / [6]: only with select low / high
			an_mask[port[1:0]] = an_mask[port[1:0]] | (fm << lsb[2:0]);
			an_val[port[1:0]]  = an_val[port[1:0]]  | ((v & fm) << lsb[2:0]);
		end
	end
end

reg [7:0] in_port[4];
always @(posedge CLK_40M) begin
	for (int p = 0; p < 4; p++)
		for (int b = 0; b < 8; b++)
			in_port[p][b] <= an_mask[p][b] ? an_val[p][b] : dip_sw[p][b] ^ ctl[in_map[p*8 + b]];
end

// Light gun (claybust / gunchamp): crosshair moved by the mouse, the left stick or the D-pad; fire or the left
// mouse button pulls the trigger. Bitmap x 0-255, picture row 0-223; CRT Flip turns the controls with the picture.
reg  [7:0] gun_x = 8'd128, gun_y = 8'd112;
reg        mouse_tog = 1'b0, gun_vbl = 1'b0;
wire signed [9:0] mouse_dx = {ps2_mouse[4], ps2_mouse[4], ps2_mouse[15:8]};
wire signed [9:0] mouse_dy = -{ps2_mouse[5], ps2_mouse[5], ps2_mouse[23:16]};          // PS/2 Y is positive upward
wire signed [7:0] ana_x = joy_la0[7:0], ana_y = joy_la0[15:8];
wire signed [9:0] stick_dx = (ana_x > 8'sd12 || ana_x < -8'sd12) ? (10'(ana_x) >>> 4) :
                             dir1[0] ? 10'sd3 : dir1[1] ? -10'sd3 : 10'sd0;
wire signed [9:0] stick_dy = (ana_y > 8'sd12 || ana_y < -8'sd12) ? (10'(ana_y) >>> 4) :
                             dir1[2] ? 10'sd3 : dir1[3] ? -10'sd3 : 10'sd0;
wire signed [9:0] gun_dx = status[22] ? -(mouse_tog != ps2_mouse[24] ? mouse_dx : stick_dx) :
                                         (mouse_tog != ps2_mouse[24] ? mouse_dx : stick_dx);
wire signed [9:0] gun_dy = status[22] ? -(mouse_tog != ps2_mouse[24] ? mouse_dy : stick_dy) :
                                         (mouse_tog != ps2_mouse[24] ? mouse_dy : stick_dy);
wire signed [10:0] gun_nx = $signed({3'b000, gun_x}) + gun_dx;
wire signed [10:0] gun_ny = $signed({3'b000, gun_y}) + gun_dy;
always @(posedge CLK_40M) begin
	mouse_tog <= ps2_mouse[24];
	gun_vbl   <= vblank;
	if (mouse_tog != ps2_mouse[24] || (vblank && !gun_vbl)) begin
		gun_x <= gun_nx < 0 ? 8'd0 : gun_nx > 255 ? 8'd255 : gun_nx[7:0];
		gun_y <= gun_ny < 0 ? 8'd0 : gun_ny > 223 ? 8'd223 : gun_ny[7:0];
	end
end
wire gun_trig = joystick_0[4] | kb_b1 | ps2_mouse[0];
assign gx = 8'((({1'b0, gun_x} + 9'd4) * 17'd252) >> 8);   // picture x 0-259 -> 0-255
assign gy = 8'(({1'b0, gun_y} * 17'd293) >> 8);            // row 0-223 -> 0-255

// PAUSE SYSTEM
wire [23:0] rgb_out;
pause #(8,8,8,40) pause
(
	.*,
	.clk_sys(CLK_40M),
	.user_button(m_pause),
	.pause_request(hs_pause & hs_configured),   // an MRA without hiscore config must never pause (and mute) the core
	.options(~status[20:19])
);

///////////////                 Video                  ////////////////

wire hs, vs;
wire [7:0] r, g, b;
wire ce_pix;
wire b8_hs, b8_vs, b8_hb, b8_vb, b8_ce;
wire [7:0] b8_r, b8_g, b8_b;
wire z_hs, z_vs, z_hb, z_vb, z_ce, z_on;
assign {hs, vs, hblank, vblank, ce_pix} = zac ? {z_hs, z_vs, z_hb, z_vb, z_ce} : {b8_hs, b8_vs, b8_hb, b8_vb, b8_ce};
wire [7:0] z_r, z_g, z_b;
wire [7:0] b8_snd1;
wire [31:0] b8_lat;                 // sound latches 4-1
assign b8_lat[7:0] = b8_snd1;
assign an_sel = an_cfg[96][7] ? b8_lat[{an_cfg[96][5:4], an_cfg[96][2:0]}] : b8_snd1[1];
assign {r, g, b} = zac ? {z_r, z_g, z_b} : {b8_r, b8_g, b8_b};

// discrete sound engine: the set's sound board program arrives as MRA index 5 (tools/dsnd_boards.py)
reg ds_on_r = 1'b0;
always @(posedge CLK_40M) if (ioctl_wr) begin
	if (ioctl_index == 8'd1 && ioctl_addr == 25'd0) ds_on_r <= 1'b0;
	if (ioctl_index == 8'd5) ds_on_r <= 1'b1;
end
assign ds_on = ds_on_r;

wire [47:0] ds_src = {7'd0, vblank, 8'd0, b8_lat};     // misc, latch 0, latches 4-1

dsnd_engine dsnd
(
	.clk(CLK_40M),
	.reset(reset),
	.pause(pause_cpu),
	.prog_wr(ioctl_wr && ioctl_index == 8'd5),
	.prog_addr(ioctl_addr[11:0]),
	.prog_data(ioctl_dout),
	.src(ds_src),
	.out(ds_audio)
);

wire rotate_ccw = ~game_flags[7];  // ROT270 sets rotate CCW, ROT90 sets CW
wire no_rotate  = ~game_vert | status[12] | direct_video;
wire flip       = status[11];
screen_rotate screen_rotate(.*);

arcade_video #(720,24) arcade_video                 // Zaccaria: 720 master clocks per visible line
(
	.*,

	.clk_video(CLK_40M),

	.RGB_in(rgb_out),
	.HBlank(hblank),
	.VBlank(vblank),
	.HSync(hs),
	.VSync(vs),

	.fx(status[18:16])
);

///////////////                 Board                  ////////////////

mw8080_board board
(
	.clk(CLK_40M),
	.reset(reset | zac),
	.pause(pause_cpu),
	.variant(game_var),

	.in0(in_port[0]),
	.in1(in_port[1]),
	.in2(in_port[2]),
	.in3(in_port[3]),
	.cocktail(1'b0),
	.taito_snd(snd_flags[0]),
	.rom_dec(rom_dec[2:0]),
	.gun_x(gun_x),
	.gun_y(gun_y),
	.gun_trig(gun_trig),
	.xhair_en(~status[24]),
	.f3(f3_cfg),

	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),

	.crt_flip(status[22]),
	.ov_en(~status[23]),
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
	.snd2(b8_lat[15:8]),
	.snd34(b8_lat[31:16]),
	.audio(b8_audio),
	.audio_r(b8_audio_r),
	.io_tab(io_tab),

	.hs_address(hs_address),
	.hs_data_in(hs_data_in),
	.hs_data_out(hs_data_out),
	.hs_write(hs_write_enable & hs_configured)
);

zac1b1120_board zboard
(
	.clk(CLK_40M),
	.reset(reset | ~zac),
	.pause(pause_cpu),
	.dodgem(game_var == 8'd33),

	.in0(in_port[0]),
	.in1(in_port[1]),
	.in2(in_port[2]),
	.in3(in_port[3]),

	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_wr0(ioctl_wr & (ioctl_index == 8'd0)),

	.ov_en(~status[23]),
	.ov_tab(ov_tab),

	.ce_pix(z_ce),
	.video_on(z_on),
	.video_r(z_r),
	.video_g(z_g),
	.video_b(z_b),
	.video_hs(z_hs),
	.video_vs(z_vs),
	.video_hblank(z_hb),
	.video_vblank(z_vb),

	.audio(z_audio)
);

// Hiscore: config = MRA index 3, dump = index 4; RAM via the board's second port while the CPU is paused
wire [15:0] hs_address;
wire  [7:0] hs_data_in;
wire  [7:0] hs_data_out;
wire        hs_write_enable;
wire        hs_access_read;
wire        hs_access_write;
wire        hs_pause;
wire        hs_configured;

hiscore #(
	.HS_ADDRESSWIDTH(16),
	.CFG_ADDRESSWIDTH(4),
	.CFG_LENGTHWIDTH(2)
) hi (
	.*,
	.clk(CLK_40M),
	.paused(pause_cpu),
	.autosave(status[21]),
	.ram_address(hs_address),
	.data_from_ram(hs_data_out),
	.data_to_ram(hs_data_in),
	.data_from_hps(ioctl_dout),
	.data_to_hps(ioctl_din),
	.ram_write(hs_write_enable),
	.ram_intent_read(hs_access_read),
	.ram_intent_write(hs_access_write),
	.pause_cpu(hs_pause),
	.configured(hs_configured)
);

endmodule
