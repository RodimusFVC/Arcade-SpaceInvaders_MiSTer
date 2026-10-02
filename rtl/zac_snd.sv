//============================================================================
//
//  Zaccaria The Invaders sound board: SN76477 with latch-switched components, 555 VCO sweep, S2636 tone filter
//  Copyright (C) 2026 Rodimus
//
//  Circuit from the Zaccaria sound board schematic ("The Invaders"); SN76477 behaviour (rate formulas, thresholds,
//  noise generator, output gain tables) follows MAME devices/sound/sn76477.cpp (Zsolt Vasvari, Derrick Renaud).
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

// clk = 39.936 MHz, one update every 208 clocks (192 kHz). Voltages Q24. Constants: tools/zac_snd_consts.py.
// latch (1E80, 74174 CI6): [0] enable (1 = on; 0 -> 1 fires the one-shot), [1]..[5] component switches, [7] mixer
// noise (else VCO). Bit 6 is not on this board.

module zac_snd
(
    input  logic        clk,
    input  logic        reset,
    input  logic        pause,
    input  logic  [7:0] latch,
    input  logic        pvi,            // S2636 sound output (square wave)
    input  logic [10:0] tweak,          // Game Audio: {aged caps, oscillator, filter, timing} settings
    output logic signed [15:0] out
);

`include "zac_snd_consts.svh"


// Game Audio (OSD, per game): capacitor groups scale the rate constants that depend on them. tweak = {oscillator,
// filter, timing} index: 0 Factory, 1-3 = -10 / -20 / -30 %, 4-6 = +10 / +20 / +30 % capacitance (rate x 1 / C).
// timing: one-shot, attack / decay; oscillator: SLF, VCO, 555 sweep, noise clock; filter: noise filter, PVI filter.
localparam int NK = 42;
localparam int KO_OS_CHG      = 0;
localparam int KO_OS_DIS      = 8;
localparam int KO_AD_DIS      = 12;
localparam int KO_SLF_CHG     = 20;
localparam int KO_SLF_DIS     = 22;
localparam int KO_VCO_STEP    = 24;
localparam int KO_NF_CHG      = 26;
localparam int KO_NF_DIS      = 30;
localparam int KO_AD_CHG      = 34;
localparam int KO_K555_CHG    = 35;
localparam int KO_K555_DIS    = 36;
localparam int KO_NOISE_FREQ  = 37;
localparam int KO_K_PVI1_LO   = 38;
localparam int KO_K_PVI1_HI   = 39;
localparam int KO_K_PVI2      = 40;
localparam int KO_K_PVI_HP    = 41;
localparam logic [2*NK-1:0] KG = 84'b010101011010100001010101010101011010101010100000000000000000000000000000000000000000;      // 0 timing, 1 filter, 2 oscillator
localparam logic [NK-1:0]   KC = 42'b111101100000000000000000000000000000000000;        // 1 = RC coefficient, capped at 1.0
localparam int KB0 = ZS_OS_CHG[0];
localparam int KB1 = ZS_OS_CHG[1];
localparam int KB2 = ZS_OS_CHG[2];
localparam int KB3 = ZS_OS_CHG[3];
localparam int KB4 = ZS_OS_CHG[4];
localparam int KB5 = ZS_OS_CHG[5];
localparam int KB6 = ZS_OS_CHG[6];
localparam int KB7 = ZS_OS_CHG[7];
localparam int KB8 = ZS_OS_DIS[0];
localparam int KB9 = ZS_OS_DIS[1];
localparam int KB10 = ZS_OS_DIS[2];
localparam int KB11 = ZS_OS_DIS[3];
localparam int KB12 = ZS_AD_DIS[0];
localparam int KB13 = ZS_AD_DIS[1];
localparam int KB14 = ZS_AD_DIS[2];
localparam int KB15 = ZS_AD_DIS[3];
localparam int KB16 = ZS_AD_DIS[4];
localparam int KB17 = ZS_AD_DIS[5];
localparam int KB18 = ZS_AD_DIS[6];
localparam int KB19 = ZS_AD_DIS[7];
localparam int KB20 = ZS_SLF_CHG[0];
localparam int KB21 = ZS_SLF_CHG[1];
localparam int KB22 = ZS_SLF_DIS[0];
localparam int KB23 = ZS_SLF_DIS[1];
localparam int KB24 = ZS_VCO_STEP[0];
localparam int KB25 = ZS_VCO_STEP[1];
localparam int KB26 = ZS_NF_CHG[0];
localparam int KB27 = ZS_NF_CHG[1];
localparam int KB28 = ZS_NF_CHG[2];
localparam int KB29 = ZS_NF_CHG[3];
localparam int KB30 = ZS_NF_DIS[0];
localparam int KB31 = ZS_NF_DIS[1];
localparam int KB32 = ZS_NF_DIS[2];
localparam int KB33 = ZS_NF_DIS[3];
localparam int KB34 = ZS_AD_CHG;
localparam int KB35 = ZS_K555_CHG;
localparam int KB36 = ZS_K555_DIS;
localparam int KB37 = ZS_NOISE_FREQ;
localparam int KB38 = ZS_K_PVI1_LO;
localparam int KB39 = ZS_K_PVI1_HI;
localparam int KB40 = ZS_K_PVI2;
localparam int KB41 = ZS_K_PVI_HP;
logic signed [31:0] kr [NK];
logic  [6:0] ks = 7'd0;
logic [10:0] tw_d = 11'h7FF;
logic        krun = 1'b1;

function automatic signed [31:0] kb(input [6:0] i);
    case (i)
        7'd0: kb = KB0;
        7'd1: kb = KB1;
        7'd2: kb = KB2;
        7'd3: kb = KB3;
        7'd4: kb = KB4;
        7'd5: kb = KB5;
        7'd6: kb = KB6;
        7'd7: kb = KB7;
        7'd8: kb = KB8;
        7'd9: kb = KB9;
        7'd10: kb = KB10;
        7'd11: kb = KB11;
        7'd12: kb = KB12;
        7'd13: kb = KB13;
        7'd14: kb = KB14;
        7'd15: kb = KB15;
        7'd16: kb = KB16;
        7'd17: kb = KB17;
        7'd18: kb = KB18;
        7'd19: kb = KB19;
        7'd20: kb = KB20;
        7'd21: kb = KB21;
        7'd22: kb = KB22;
        7'd23: kb = KB23;
        7'd24: kb = KB24;
        7'd25: kb = KB25;
        7'd26: kb = KB26;
        7'd27: kb = KB27;
        7'd28: kb = KB28;
        7'd29: kb = KB29;
        7'd30: kb = KB30;
        7'd31: kb = KB31;
        7'd32: kb = KB32;
        7'd33: kb = KB33;
        7'd34: kb = KB34;
        7'd35: kb = KB35;
        7'd36: kb = KB36;
        7'd37: kb = KB37;
        7'd38: kb = KB38;
        7'd39: kb = KB39;
        7'd40: kb = KB40;
        7'd41: kb = KB41;
        default: kb = 0;
    endcase
endfunction

function automatic [16:0] aged_f(input [1:0] i);   // Aged Caps Off / Light / Heavy: capacitance x 1, 0.85, 0.7
    aged_f = i == 2'd1 ? 17'd77101 : i == 2'd2 ? 17'd93623 : 17'd65536;
endfunction

function automatic [16:0] cap_f(input [2:0] i);
    case (i)
        3'd1: cap_f = 17'd72818;   3'd2: cap_f = 17'd81920;   3'd3: cap_f = 17'd93623;
        3'd4: cap_f = 17'd59578;   3'd5: cap_f = 17'd54613;   3'd6: cap_f = 17'd50412;
        default: cap_f = 17'd65536;
    endcase
endfunction

always_ff @(posedge clk) begin
    if (reset || tweak != tw_d) begin
        tw_d <= tweak;
        ks   <= 7'd0;
        krun <= 1'b1;
    end else if (krun) begin : scale
        logic [1:0] g;
        logic signed [51:0] pr;
        logic        [33:0] fa;
        logic signed [31:0] v;
        g  = KG[ks*2 +: 2];
        fa = cap_f(g == 2'd0 ? tweak[2:0] : g == 2'd1 ? tweak[5:3] : tweak[8:6]) * (g == 2'd2 ? 17'd65536 : aged_f(tweak[10:9]));
        pr = kb(ks) * $signed({1'b0, fa[33:16]});
        v  = 32'(pr >>> 16);
        kr[ks] <= (KC[ks] && v > 32'sd16777216) ? 32'sd16777216 : v;
        if (ks == 7'(NK - 1)) krun <= 1'b0;
        else                  ks <= ks + 7'd1;
    end
end

logic [7:0] tdiv = 8'd0;
wire        tick = (tdiv == 8'd207) & ~pause;
always_ff @(posedge clk) tdiv <= (tdiv == 8'd207) ? 8'd0 : tdiv + 8'd1;

wire en   = latch[0];
wire b1   = latch[1], b2 = latch[2], b3 = latch[3], b4 = latch[4], b5 = latch[5];
wire b15  = b1 | b5, b23 = b2 | b3;

logic        en_d = 1'b0;
logic signed [31:0] os_v = 0, slf_v = 0, vco_v = 0, nf_v = 0, ad_v = 0, v555 = 0;
logic        os_run = 1'b0, slf_ff = 1'b0, vco_ff = 1'b0, nbit = 1'b0, fbit = 1'b0, o555 = 1'b1;
logic [30:0] rng = 31'd0;
logic signed [31:0] ncount = 0;
logic signed [31:0] p1 = ZS_PVI_HI, p2 = ZS_PVI_HI, php = ZS_PVI_HI;   // the tone path idles at 5 V
logic signed [15:0] sn_out = 0;

// multiply a Q24 voltage difference by a Q24 fraction
function automatic logic signed [31:0] mulq(input logic signed [31:0] d, input int k);
    logic signed [63:0] p;
    p = 64'(d) * 64'(k);
    mulq = 32'(p >>> 24);
endfunction

always_ff @(posedge clk) begin
    if (reset) begin
        en_d <= 1'b0; os_v <= 0; slf_v <= 0; vco_v <= 0; nf_v <= 0; ad_v <= 0; v555 <= 0;
        os_run <= 1'b0; slf_ff <= 1'b0; vco_ff <= 1'b0; nbit <= 1'b0; fbit <= 1'b0; o555 <= 1'b1;
        rng <= 31'd0; ncount <= 0; p1 <= ZS_PVI_HI; p2 <= ZS_PVI_HI; php <= ZS_PVI_HI; sn_out <= 0;
    end else if (tick) begin : upd
        logic signed [31:0] os_n, slf_n, vco_n, nf_n, ad_n, v555_n, ext, vmax;
        logic        start, run, slf_ffn, vco_ffn, fbit_n, rb, out_bit;
        logic [5:0]  idx;

        // enable (pin 9) falling edge = latch bit 0 rising: attack restarts, the one-shot runs
        start = en & ~en_d;
        run   = start | os_run;
        en_d <= en;

        // one-shot (pin 23 held at 0 V by the board while disabled)
        if (!en)      os_n = 0;
        else if (run) os_n = (os_v + kr[KO_OS_CHG + {b23, b2, b15}] > ZS_ONE_SHOT_MAX) ? ZS_ONE_SHOT_MAX : os_v + kr[KO_OS_CHG + {b23, b2, b15}];
        else          os_n = (os_v - kr[KO_OS_DIS + {b2, b15}] < 0) ? 0 : os_v - kr[KO_OS_DIS + {b2, b15}];
        if (os_n >= ZS_ONE_SHOT_MAX) run = 1'b0;
        os_v   <= os_n;
        os_run <= en & run;

        // SLF
        slf_n = !slf_ff ? ((slf_v + kr[KO_SLF_CHG + b1] > ZS_SLF_MAX) ? ZS_SLF_MAX : slf_v + kr[KO_SLF_CHG + b1])
                        : ((slf_v - kr[KO_SLF_DIS + b1] < ZS_SLF_MIN) ? ZS_SLF_MIN : slf_v - kr[KO_SLF_DIS + b1]);
        slf_ffn = (slf_n >= ZS_SLF_MAX) ? 1'b1 : (slf_n <= ZS_SLF_MIN) ? 1'b0 : slf_ff;
        slf_v  <= slf_n;
        slf_ff <= slf_ffn;

        // VCO: external voltage (pin 16, DB3) or the SLF; 50 % duty
        ext  = (v555 > ZS_V555_LO) ? v555 - ZS_V555_LO : 0;                    // 555 sawtooth, clamped at 0 V
        vmax = (b3 ? ext : slf_n) + ZS_VCO_DIFF;
        vco_n = !vco_ff ? ((vco_v + kr[KO_VCO_STEP + b3] > vmax) ? vmax : vco_v + kr[KO_VCO_STEP + b3])
                        : ((vco_v - kr[KO_VCO_STEP + b3] < ZS_VCO_MIN) ? ZS_VCO_MIN : vco_v - kr[KO_VCO_STEP + b3]);
        vco_ffn = (vco_n >= vmax) ? 1'b1 : (vco_n <= ZS_VCO_MIN) ? 1'b0 : vco_ff;
        vco_v  <= vco_n;
        vco_ff <= vco_ffn;

        // noise generator (MAME generate_next_real_noise_bit), clocked at kr[KO_NOISE_FREQ]
        rb = nbit;
        if (ncount <= kr[KO_NOISE_FREQ]) begin
            rb = rng[28] ^ rng[0];
            if ({rng[28], rng[4:0]} == 6'd0) rb = 1'b1;
            rng    <= {rb, rng[30:1]};
            ncount <= ncount + ZS_FS - kr[KO_NOISE_FREQ];
        end else
            ncount <= ncount - kr[KO_NOISE_FREQ];
        nbit <= rb;

        // noise filter
        nf_n = rb ? ((nf_v + kr[KO_NF_CHG + {b23, b4}] > ZS_NOISE_MAX) ? ZS_NOISE_MAX : nf_v + kr[KO_NF_CHG + {b23, b4}])
                  : ((nf_v - kr[KO_NF_DIS + {b23, b4}] < 0) ? 0 : nf_v - kr[KO_NF_DIS + {b23, b4}]);
        fbit_n = (nf_n >= ZS_NOISE_HI) ? 1'b0 : (nf_n <= ZS_NOISE_LO) ? 1'b1 : fbit;
        nf_v <= nf_n;
        fbit <= fbit_n;

        // attack / decay (one-shot envelope): charging while the one-shot runs
        ad_n = start ? 0 : ad_v;
        ad_n = run ? ((ad_n + kr[KO_AD_CHG] > ZS_AD_MAX) ? ZS_AD_MAX : ad_n + kr[KO_AD_CHG])
                   : ((ad_n - kr[KO_AD_DIS + {b2, b3, b15}] < 0) ? 0 : ad_n - kr[KO_AD_DIS + {b2, b3, b15}]);
        ad_v <= ad_n;

        // output (MAME: enabled and the VCO not saturated); mixer B (latch bit 7) = noise, else VCO
        idx = 6'((64'(ad_n) * 10) >>> 24);
        if (idx > 6'd44) idx = 6'd44;
        out_bit = latch[7] ? fbit_n : vco_ffn;
        sn_out <= (en && vco_n <= ZS_VCO_MAX) ? 16'(out_bit ? ZS_OUT_P[idx] : ZS_OUT_N[idx]) : 16'sd0;

        // 555 (CI5): its timing capacitor is the VCO sweep
        if (o555) begin
            v555_n = v555 + mulq(ZS_V555 - v555, kr[KO_K555_CHG]);
            if (v555_n >= ZS_V555_HI) o555 <= 1'b0;
        end else begin
            v555_n = v555 - mulq(v555, kr[KO_K555_DIS]);
            if (v555_n <= ZS_V555_LO) o555 <= 1'b1;
        end
        v555 <= v555_n;

        // S2636 tone: 7406 inverter into a two-pole RC low-pass, then AC coupled
        p1  <= p1 + mulq((pvi ? ZS_PVI_LO : ZS_PVI_HI) - p1, pvi ? kr[KO_K_PVI1_LO] : kr[KO_K_PVI1_HI]);
        p2  <= p2 + mulq(p1 - p2, kr[KO_K_PVI2]);
        php <= php + mulq(p2 - php, kr[KO_K_PVI_HP]);
    end
end

// mix: SN76477 output + filtered PVI tone (P1 balance: ZS_PVI_GAIN per volt)
wire signed [31:0] pdiff = p2 - php;
wire signed [33:0] mix   = 34'(sn_out) + 34'(mulq(pdiff, ZS_PVI_GAIN));
assign out = mix > 34'sd32767 ? 16'sd32767 : mix < -34'sd32768 ? -16'sd32768 : 16'(mix);

endmodule
