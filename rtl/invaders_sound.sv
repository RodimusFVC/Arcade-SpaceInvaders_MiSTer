//============================================================================
//
//  Space Invaders sound board (Midway A084-90700-B000 / Taito L-shaped)
//  Copyright (C) 2026 Rodimus
//
//  Circuit model after MAME invaders_discrete (Derrick Renaud) and the MAME
//  SN76477 (Zsolt Vasvari), component values from the Midway schematic
//  (tools/snd_consts.py). Bit-exact to verilator/snd/model.h.
//
//  One 48 kHz sample per 832 clocks of 39.936 MHz: a sequenced datapath with
//  one shared multiplier (operands registered, product the next clock, used
//  the clock after). Op-amp oscillators and the SN76477 VCO run 16 substeps
//  per sample. Voltages Q20, coefficients Q28.
//
//============================================================================

module invaders_sound
(
    input                    clk,            // 39.936 MHz
    input                    reset,
    input                    pause,          // freezes the board; the output holds its last sample
    input              [7:0] p1,             // port 3
    input              [7:0] p2,             // port 5
    input                    v16,            // 16V: the bonus base's "480 Hz"
    input                    taito,          // Taito L-shaped sound board values (else Midway)
    output reg signed [15:0] out = 16'sd0
);

`include "invaders_snd_consts.svh"

function signed [35:0] pos(input signed [35:0] v);
    pos = v < 0 ? 36'sd0 : v;
endfunction

function signed [35:0] clamp(input signed [35:0] v, input signed [35:0] lo, input signed [35:0] hi);
    clamp = v < lo ? lo : v > hi ? hi : v;
endfunction

function signed [35:0] fl_kc(input [3:0] d);
    case (d)
        4'd0: fl_kc = FL_KC0;   4'd1: fl_kc = FL_KC1;   4'd2: fl_kc = FL_KC2;   4'd3: fl_kc = FL_KC3;
        4'd4: fl_kc = FL_KC4;   4'd5: fl_kc = FL_KC5;   4'd6: fl_kc = FL_KC6;   4'd7: fl_kc = FL_KC7;
        4'd8: fl_kc = FL_KC8;   4'd9: fl_kc = FL_KC9;   4'd10: fl_kc = FL_KC10; 4'd11: fl_kc = FL_KC11;
        4'd12: fl_kc = FL_KC12; 4'd13: fl_kc = FL_KC13; 4'd14: fl_kc = FL_KC14; default: fl_kc = FL_KC15;
    endcase
endfunction

// shared multiplier
reg  signed [35:0] ma = 36'sd0, mb = 36'sd0;
reg  signed [71:0] mp = 72'sd0;
always @(posedge clk) mp <= ma * mb;
wire signed [35:0] mq = 36'(mp >>> 28);

// sample timing
reg  [9:0] tick = 10'd0;
wire       go = (tick == 10'd831);
always @(posedge clk) if (reset) tick <= 10'd0; else if (!pause) tick <= go ? 10'd0 : tick + 10'd1;

// state
reg  [31:0] nz_ph = 32'd0;
reg  [16:0] lfsr = 17'h1FFFF;
reg  signed [35:0] osc_v[2];                 // saucer hit, invader hit
reg        [1:0] osc_ff = 2'b11;
reg  signed [35:0] vco_v[3];                 // saucer hit, invader hit, missile
reg        [2:0] vco_ff = 3'b100;
reg  signed [35:0] os_vc1[3], os_vc2[3];     // invader hit, explosion, missile one-shots
reg        [2:0] os_on = 3'b000;
reg              os_skip = 1'b0;             // one-shot C1 discharged at once through the diode
reg  signed [35:0] sh_env = 0, ih_env = 0, ms_env = 0;
reg  signed [35:0] fl_v = 0, fl_rc1 = 0, fl_rc2 = 0, bn_v = 0;
reg              fl_ff = 1'b1, bn_ff = 1'b1;
reg  signed [35:0] ex_vc = 0, ex_rc1 = 0, ex_rc2 = 0, ms_cr = 0, ms_vc = 0;
reg  signed [35:0] sn_slf = 0, sn_vco = 0;
reg              sn_slf_ff = 1'b0, sn_vco_ff = 1'b0;
reg  signed [35:0] mx_c11 = 0, mx_c44 = 0, mx_amp = 0;

integer n;
initial begin
    for (n = 0; n < 2; n = n + 1) osc_v[n] = 36'sd0;
    for (n = 0; n < 3; n = n + 1) begin vco_v[n] = 36'sd0; os_vc1[n] = 36'sd0; os_vc2[n] = 36'sd0; end
end

// program registers
reg  [5:0] st = 6'd63;
reg  [4:0] sub = 5'd0;
reg  [1:0] k = 2'd0;
reg  [7:0] s_p1 = 8'd0, s_p2 = 8'd0;
reg        s_v16 = 1'b0;
reg  signed [35:0] nz, sd0, sd1, id0, id1, md0, md1, lhs, t1, m73, m74, m77, acc, v;
reg  signed [35:0] o_sh, o_fl, o_bn, o_ih, o_ex, o_ms, o_sn;
reg  [4:0] cnt_sh, cnt_ms, cnt_sn;

wire signed [35:0] os_t = (k == 2'd0 ? s_p1[3] : k == 2'd1 ? s_p1[2] : s_p1[1]) ? OS_TRIG : 36'sd0;
wire signed [35:0] os_e1c = k == 2'd0 ? IH_E1C : k == 2'd1 ? EX_E1C : MS_E1C;
wire signed [35:0] os_e1d = k == 2'd0 ? IH_E1D : k == 2'd1 ? EX_E1D : MS_E1D;

localparam S_START = 6'd0,  S_OSC = 6'd1,  S_OS = 6'd20, S_IH = 6'd28, S_EX = 6'd31, S_MS = 6'd38,
           S_MX = 6'd50;

always @(posedge clk) begin
    if (reset) begin
        st <= 6'd63;
        nz_ph <= 32'd0; lfsr <= 17'h1FFFF;
        for (n = 0; n < 2; n = n + 1) osc_v[n] <= 36'sd0;
        for (n = 0; n < 3; n = n + 1) begin vco_v[n] <= 36'sd0; os_vc1[n] <= 36'sd0; os_vc2[n] <= 36'sd0; end
        osc_ff <= 2'b11; vco_ff <= 3'b100; os_on <= 3'b000;
        sh_env <= 0; ih_env <= 0; ms_env <= 0; fl_v <= 0; fl_rc1 <= 0; fl_rc2 <= 0; bn_v <= 0;
        fl_ff <= 1'b1; bn_ff <= 1'b1; ex_vc <= 0; ex_rc1 <= 0; ex_rc2 <= 0; ms_cr <= 0; ms_vc <= 0;
        sn_slf <= 0; sn_vco <= 0; sn_slf_ff <= 1'b0; sn_vco_ff <= 1'b0; mx_c11 <= 0; mx_c44 <= 0; mx_amp <= 0;
        out <= 16'sd0;
    end
    else begin
        if (go && !pause) begin
            s_p1 <= p1; s_p2 <= p2; s_v16 <= v16;
            st <= S_START;
        end
        else case (st)
            // noise; saucer hit envelope
            S_START: begin : s_start
                reg [32:0] ph;
                reg [16:0] l;
                ph = {1'b0, nz_ph} + {1'b0, NZ_INC[31:0]};
                l = ph[32] ? {lfsr[15:0], lfsr[4] ^ lfsr[16]} : lfsr;
                nz_ph <= ph[31:0];
                lfsr <= l;
                nz <= l[12] ? V12 : 36'sd0;
                sh_env <= clamp(sh_env + (s_p2[4] ? SH_ENV_UP : 36'sd0) - SH_ENV_DN, 36'sd0, VOH);
                sub <= 5'd0;
                cnt_sh <= 5'd0; cnt_ms <= 5'd0; cnt_sn <= 5'd0;
                st <= S_OSC;
            end
            // saucer hit and invader hit oscillators (constant charge), 16 substeps
            S_OSC: begin
                for (n = 0; n < 2; n = n + 1) begin : osc_sub
                    reg signed [35:0] d0, d1, x;
                    d0 = n == 0 ? SH_OSC_D0 : IH_OSC_D0;
                    d1 = n == 0 ? SH_OSC_D1 : IH_OSC_D1;
                    if (osc_ff[n]) begin
                        x = osc_v[n] + d1;
                        if (x > OSC_TH) begin osc_v[n] <= OSC_TH; osc_ff[n] <= 1'b0; end else osc_v[n] <= x;
                    end
                    else begin
                        x = osc_v[n] - d0;
                        if (x < OSC_TL) begin osc_v[n] <= OSC_TL; osc_ff[n] <= 1'b1; end else osc_v[n] <= x;
                    end
                end
                sub <= sub + 5'd1;
                if (sub == 5'd15) st <= 6'd2;
            end
            6'd2:  begin ma <= osc_v[0] - VBE; mb <= SH_VCO_KA; st <= 6'd3; end
            6'd3:  begin ma <= osc_v[0] - VBE; mb <= SH_VCO_KB; st <= 6'd4; end
            6'd4:  begin sd0 <= mq; ma <= osc_v[1] - VBE; mb <= IH_VCO_KA; st <= 6'd5; end
            6'd5:  begin sd1 <= mq; ma <= osc_v[1] - VBE; mb <= IH_VCO_KB; st <= 6'd6; end
            6'd6:  begin id0 <= mq; st <= 6'd7; end
            6'd7:  begin id1 <= mq; sub <= 5'd0; st <= 6'd8; end
            // saucer hit and invader hit VCOs (VCO_1: charge while ff = 0), 16 substeps
            6'd8: begin
                for (n = 0; n < 2; n = n + 1) begin : vco_sub
                    reg signed [35:0] d0, d1, x;
                    reg f;
                    d0 = n == 0 ? sd0 : id0;
                    d1 = n == 0 ? sd1 : id1;
                    f = vco_ff[n];
                    if (!f) begin
                        x = vco_v[n] + d1;
                        if (x > VCO_TH) begin vco_v[n] <= VCO_TH; f = 1'b1; end else vco_v[n] <= x;
                    end
                    else begin
                        x = vco_v[n] - d0;
                        if (x < VCO_TL) begin vco_v[n] <= VCO_TL; f = 1'b0; end else vco_v[n] <= x;
                    end
                    vco_ff[n] <= f;
                    if (n == 0) cnt_sh <= cnt_sh + {4'd0, f};
                end
                sub <= sub + 5'd1;
                if (sub == 5'd15) st <= 6'd9;
            end
            // saucer hit output; fleet 555
            6'd9: begin : s9
                reg signed [35:0] sq;
                sq = 36'((40'(cnt_sh) * 40'(VOH)) >>> 4);
                o_sh <= clamp(pos(sh_env - VBE) - pos(sq - VBE) - SH_OUT_C, 36'sd0, VOH);
                ma <= (fl_ff && s_p2[3:0] != 4'd0) ? FL_VCH - fl_v : fl_v;
                mb <= fl_ff ? fl_kc(s_p2[3:0]) : FL_KD;
                st <= 6'd10;
            end
            6'd10: st <= 6'd11;
            6'd11: begin : s11
                reg signed [35:0] x;
                reg f;
                x = (fl_ff && s_p2[3:0] != 4'd0) ? fl_v + mq : fl_v - mq;
                f = fl_ff;
                if (fl_ff && x >= V555_TH) begin x = V555_TH; f = 1'b0; end
                else if (!fl_ff && x <= V555_TR) begin x = V555_TR; f = 1'b1; end
                fl_v <= x; fl_ff <= f;
                ma <= (f ? VTTL : 36'sd0) - fl_rc1; mb <= FL_RC1;
                st <= 6'd12;
            end
            6'd12: st <= 6'd13;
            6'd13: begin fl_rc1 <= fl_rc1 + mq; ma <= fl_rc1 + mq - fl_rc2; mb <= FL_RC2; st <= 6'd14; end
            6'd14: st <= 6'd15;
            6'd15: begin
                fl_rc2 <= fl_rc2 + mq; o_fl <= fl_rc2 + mq;
                ma <= bn_ff ? V5 - bn_v : bn_v; mb <= bn_ff ? BN_KC : BN_KD;
                st <= 6'd16;
            end
            6'd16: st <= 6'd17;
            // bonus missile base
            6'd17: begin : s17
                reg signed [35:0] x;
                reg f;
                if (!s_p1[4]) begin x = 36'sd0; f = 1'b1; end
                else begin
                    x = bn_ff ? bn_v + mq : bn_v - mq;
                    f = bn_ff;
                    if (bn_ff && x >= V555_TH) begin x = V555_TH; f = 1'b0; end
                    else if (!bn_ff && x <= V555_TR) begin x = V555_TR; f = 1'b1; end
                end
                bn_v <= x; bn_ff <= f;
                o_bn <= (s_p1[4] && s_v16 && f) ? VTTL : 36'sd0;
                k <= 2'd0;
                st <= S_OS;
            end
            // op-amp one-shot k (0 invader hit, 1 explosion, 2 missile)
            S_OS: begin ma <= os_t - os_vc2[k]; mb <= OS_K10; st <= S_OS + 6'd1; end
            S_OS + 6'd1: begin ma <= os_t - os_vc2[k]; mb <= OS_E2; st <= S_OS + 6'd2; end
            S_OS + 6'd2: begin lhs <= mq + (os_on[k] ? OS_VOUT_R5 : 36'sd0); st <= S_OS + 6'd3; end
            S_OS + 6'd3: begin : os3
                reg signed [35:0] vo, vc1;
                reg on;
                os_vc2[k] <= os_vc2[k] + mq;
                on = lhs > pos(os_vc1[k] - VBE) + OS_IFIX;
                os_on[k] <= on;
                vo = on ? VOH : 36'sd0;
                vc1 = os_vc1[k];
                os_skip <= 1'b0;
                if (vc1 > vo) begin
                    if (vc1 > vo + V06) begin os_vc1[k] <= vo + V06; os_skip <= 1'b1; end
                    ma <= vo - vc1;
                    mb <= os_e1d;
                end
                else begin
                    ma <= (on ? OS_T1 : OS_T0) - vc1;
                    mb <= os_e1c;
                end
                st <= S_OS + 6'd4;
            end
            S_OS + 6'd4: st <= S_OS + 6'd5;
            S_OS + 6'd5: begin
                if (!os_skip) os_vc1[k] <= os_vc1[k] + mq;
                st <= k == 2'd0 ? S_IH : k == 2'd1 ? S_EX : S_MS;
            end
            // invader hit output
            S_IH: begin
                ih_env <= clamp(ih_env + (os_on[0] ? IH_ENV_UP : 36'sd0) - IH_ENV_DN, 36'sd0, VOH);
                ma <= pos(vco_v[1] - VBE); mb <= IH_OUT_G;
                st <= S_IH + 6'd1;
            end
            S_IH + 6'd1: st <= S_IH + 6'd2;
            S_IH + 6'd2: begin
                o_ih <= clamp(pos(ih_env - VBE) - mq - IH_OUT_C, 36'sd0, VOH);
                k <= 2'd1;
                st <= S_OS;
            end
            // explosion
            S_EX: begin
                ma <= os_on[1] ? EX_VT - ex_vc : VBE - ex_vc; mb <= os_on[1] ? EX_EC : EX_ED;
                st <= S_EX + 6'd1;
            end
            S_EX + 6'd1: st <= S_EX + 6'd2;
            S_EX + 6'd2: begin : ex2
                reg signed [35:0] vc, o;
                vc = ex_vc + mq;
                ex_vc <= vc;
                o = pos(pos(vc - VBE) - pos(nz - VBE) - EX_C);
                if (o > VOH) o = VOH;
                ma <= o - ex_rc1; mb <= EX_RC1;
                st <= S_EX + 6'd3;
            end
            S_EX + 6'd3: st <= S_EX + 6'd4;
            S_EX + 6'd4: begin ex_rc1 <= ex_rc1 + mq; ma <= ex_rc1 + mq - ex_rc2; mb <= EX_RC2; st <= S_EX + 6'd5; end
            S_EX + 6'd5: st <= S_EX + 6'd6;
            S_EX + 6'd6: begin ex_rc2 <= ex_rc2 + mq; o_ex <= ex_rc2 + mq; k <= 2'd2; st <= S_OS; end
            // missile
            S_MS: begin : ms0
                reg signed [35:0] e, x;
                e = clamp(ms_env + (os_on[2] ? MS_ENV_UP : 36'sd0) - MS_ENV_DN, 36'sd0, VOH);
                ms_env <= e;
                m73 <= clamp(e - VBE, 36'sd0, V12);
                x = nz - ms_cr;
                m74 <= x;
                ma <= x; mb <= MS_CR;
                st <= S_MS + 6'd1;
            end
            S_MS + 6'd1: begin ma <= m74; mb <= MS_NZ_G; st <= S_MS + 6'd2; end
            S_MS + 6'd2: begin ms_cr <= ms_cr + mq; ma <= m73 - VBE; mb <= MS_VCO_K6; st <= S_MS + 6'd3; end
            S_MS + 6'd3: begin ma <= pos(mq - VBE); mb <= MS_VCO_K1; st <= S_MS + 6'd4; end
            S_MS + 6'd4: begin t1 <= mq; st <= S_MS + 6'd5; end
            S_MS + 6'd5: begin : ms5
                reg signed [35:0] d0, s;
                d0 = MS_VCO_IF + mq + t1;
                md0 <= d0; md1 <= MS_VCO_T1 - d0;
                // SN76477 SLF
                s = sn_slf_ff ? sn_slf - SN_SLF_DN : sn_slf + SN_SLF_UP;
                if (!sn_slf_ff && s > SN_SLF_MAX) s = SN_SLF_MAX;
                if (sn_slf_ff && s < SN_SLF_MIN) s = SN_SLF_MIN;
                sn_slf <= s;
                if (s >= SN_SLF_MAX) sn_slf_ff <= 1'b1; else if (s <= SN_SLF_MIN) sn_slf_ff <= 1'b0;
                sub <= 5'd0;
                st <= S_MS + 6'd6;
            end
            // missile VCO (VCO_3: charge while ff = 1) and SN76477 VCO, 16 substeps
            S_MS + 6'd6: begin : ms6
                reg signed [35:0] x, vmax;
                reg f;
                f = vco_ff[2];
                if (f) begin
                    x = vco_v[2] + md1;
                    if (x > MS_VCO_TH) begin vco_v[2] <= MS_VCO_TH; f = 1'b0; end else vco_v[2] <= x;
                end
                else begin
                    x = vco_v[2] - md0;
                    if (x < MS_VCO_TL) begin vco_v[2] <= MS_VCO_TL; f = 1'b1; end else vco_v[2] <= x;
                end
                vco_ff[2] <= f;
                cnt_ms <= cnt_ms + {4'd0, f};
                vmax = sn_slf + SN_VCO_DIFF;
                f = sn_vco_ff;
                if (!sn_vco_ff) begin x = sn_vco + SN_VCO_STEP; if (x > vmax) x = vmax; end
                else begin x = sn_vco - SN_VCO_STEP; if (x < SN_SLF_MIN) x = SN_SLF_MIN; end
                sn_vco <= x;
                if (x >= vmax) f = 1'b1; else if (x <= SN_SLF_MIN) f = 1'b0;
                sn_vco_ff <= f;
                cnt_sn <= cnt_sn + {4'd0, f};
                sub <= sub + 5'd1;
                if (sub == 5'd15) st <= S_MS + 6'd7;
            end
            S_MS + 6'd7: begin ma <= pos(m73 - VBE); mb <= MS_A3_G; st <= S_MS + 6'd8; end
            S_MS + 6'd8: st <= S_MS + 6'd9;
            S_MS + 6'd9: begin : ms9
                reg signed [35:0] sq;
                sq = 36'((40'(cnt_ms) * 40'(VOH)) >>> 4);
                m77 <= clamp(mq - pos(sq - VBE) - MS_A3_C, 36'sd0, VOH);
                ma <= s_p1[1] ? MS_VT - ms_vc : VBE - ms_vc; mb <= s_p1[1] ? MS_EC : MS_ED;
                o_sn <= s_p1[0] ? SN_LO + 36'((40'(cnt_sn) * (40'(SN_HI) - 40'(SN_LO))) >>> 4) : SN_MID;
                st <= S_MS + 6'd10;
            end
            S_MS + 6'd10: st <= S_MS + 6'd11;
            S_MS + 6'd11: begin : ms11
                reg signed [35:0] vc, o;
                vc = ms_vc + mq;
                ms_vc <= vc;
                o = pos(pos(vc - VBE) - pos(m77 - VBE) - MS_C);
                if (o > VOH) o = VOH;
                o_ms <= o;
                ma <= o - mx_c11; mb <= MX_C11;
                st <= S_MX;
            end
            // summing amplifier
            S_MX:         begin ma <= o_sh; mb <= G_SH; st <= S_MX + 6'd1; end
            S_MX + 6'd1:  begin mx_c11 <= mx_c11 + mq; ma <= o_fl; mb <= G_FL; st <= S_MX + 6'd2; end
            S_MX + 6'd2:  begin acc <= mq; ma <= o_bn; mb <= G_BN; st <= S_MX + 6'd3; end
            S_MX + 6'd3:  begin acc <= acc + mq; ma <= o_ih; mb <= G_IH; st <= S_MX + 6'd4; end
            S_MX + 6'd4:  begin acc <= acc + mq; ma <= o_ex; mb <= G_EX; st <= S_MX + 6'd5; end
            S_MX + 6'd5:  begin acc <= acc + mq; ma <= o_ms - mx_c11; mb <= G_MS; st <= S_MX + 6'd6; end
            S_MX + 6'd6:  begin acc <= acc + mq; ma <= o_sn; mb <= G_SN; st <= S_MX + 6'd7; end
            S_MX + 6'd7:  begin acc <= acc + mq; st <= S_MX + 6'd8; end
            S_MX + 6'd8:  begin
                v <= -(acc + mq);
                ma <= -(acc + mq) - mx_c44; mb <= MX_C44;
                st <= 6'd18;
            end
            6'd18: st <= 6'd19;
            // C44 couples the bus into the amp, then the first LM3900 stage runs into its rails
            6'd19: begin : mx19
                reg signed [35:0] c, x;
                c = mx_c44 + mq;
                mx_c44 <= c;
                x = clamp(v - c, MX_VLO, MX_VHI);
                v <= x;
                ma <= x - mx_amp; mb <= MX_CAMP;
                st <= S_MX + 6'd9;
            end
            S_MX + 6'd9:  st <= S_MX + 6'd10;
            S_MX + 6'd10: begin : mx10
                reg signed [35:0] a;
                a = mx_amp + mq;
                mx_amp <= a;
                ma <= v - a; mb <= OUT_G;
                st <= S_MX + 6'd11;
            end
            S_MX + 6'd11: st <= S_MX + 6'd12;
            S_MX + 6'd12: begin
                if (!s_p1[5])              out <= 16'sd0;
                else if (mq > 36'sd32767)  out <= 16'sd32767;
                else if (mq < -36'sd32768) out <= 16'h8000;
                else                       out <= 16'(mq);
                st <= 6'd63;
            end
            default: ;
        endcase
    end
end

endmodule
