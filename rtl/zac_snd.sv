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
    output logic signed [15:0] out
);

`include "zac_snd_consts.svh"

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
        else if (run) os_n = (os_v + ZS_OS_CHG[{b23, b2, b15}] > ZS_ONE_SHOT_MAX) ? ZS_ONE_SHOT_MAX : os_v + ZS_OS_CHG[{b23, b2, b15}];
        else          os_n = (os_v - ZS_OS_DIS[{b2, b15}] < 0) ? 0 : os_v - ZS_OS_DIS[{b2, b15}];
        if (os_n >= ZS_ONE_SHOT_MAX) run = 1'b0;
        os_v   <= os_n;
        os_run <= en & run;

        // SLF
        slf_n = !slf_ff ? ((slf_v + ZS_SLF_CHG[b1] > ZS_SLF_MAX) ? ZS_SLF_MAX : slf_v + ZS_SLF_CHG[b1])
                        : ((slf_v - ZS_SLF_DIS[b1] < ZS_SLF_MIN) ? ZS_SLF_MIN : slf_v - ZS_SLF_DIS[b1]);
        slf_ffn = (slf_n >= ZS_SLF_MAX) ? 1'b1 : (slf_n <= ZS_SLF_MIN) ? 1'b0 : slf_ff;
        slf_v  <= slf_n;
        slf_ff <= slf_ffn;

        // VCO: external voltage (pin 16, DB3) or the SLF; 50 % duty
        ext  = (v555 > ZS_V555_LO) ? v555 - ZS_V555_LO : 0;                    // 555 sawtooth, clamped at 0 V
        vmax = (b3 ? ext : slf_n) + ZS_VCO_DIFF;
        vco_n = !vco_ff ? ((vco_v + ZS_VCO_STEP[b3] > vmax) ? vmax : vco_v + ZS_VCO_STEP[b3])
                        : ((vco_v - ZS_VCO_STEP[b3] < ZS_VCO_MIN) ? ZS_VCO_MIN : vco_v - ZS_VCO_STEP[b3]);
        vco_ffn = (vco_n >= vmax) ? 1'b1 : (vco_n <= ZS_VCO_MIN) ? 1'b0 : vco_ff;
        vco_v  <= vco_n;
        vco_ff <= vco_ffn;

        // noise generator (MAME generate_next_real_noise_bit), clocked at ZS_NOISE_FREQ
        rb = nbit;
        if (ncount <= ZS_NOISE_FREQ) begin
            rb = rng[28] ^ rng[0];
            if ({rng[28], rng[4:0]} == 6'd0) rb = 1'b1;
            rng    <= {rb, rng[30:1]};
            ncount <= ncount + ZS_FS - ZS_NOISE_FREQ;
        end else
            ncount <= ncount - ZS_NOISE_FREQ;
        nbit <= rb;

        // noise filter
        nf_n = rb ? ((nf_v + ZS_NF_CHG[{b23, b4}] > ZS_NOISE_MAX) ? ZS_NOISE_MAX : nf_v + ZS_NF_CHG[{b23, b4}])
                  : ((nf_v - ZS_NF_DIS[{b23, b4}] < 0) ? 0 : nf_v - ZS_NF_DIS[{b23, b4}]);
        fbit_n = (nf_n >= ZS_NOISE_HI) ? 1'b0 : (nf_n <= ZS_NOISE_LO) ? 1'b1 : fbit;
        nf_v <= nf_n;
        fbit <= fbit_n;

        // attack / decay (one-shot envelope): charging while the one-shot runs
        ad_n = start ? 0 : ad_v;
        ad_n = run ? ((ad_n + ZS_AD_CHG > ZS_AD_MAX) ? ZS_AD_MAX : ad_n + ZS_AD_CHG)
                   : ((ad_n - ZS_AD_DIS[{b2, b3, b15}] < 0) ? 0 : ad_n - ZS_AD_DIS[{b2, b3, b15}]);
        ad_v <= ad_n;

        // output (MAME: enabled and the VCO not saturated); mixer B (latch bit 7) = noise, else VCO
        idx = 6'((64'(ad_n) * 10) >>> 24);
        if (idx > 6'd44) idx = 6'd44;
        out_bit = latch[7] ? fbit_n : vco_ffn;
        sn_out <= (en && vco_n <= ZS_VCO_MAX) ? 16'(out_bit ? ZS_OUT_P[idx] : ZS_OUT_N[idx]) : 16'sd0;

        // 555 (CI5): its timing capacitor is the VCO sweep
        if (o555) begin
            v555_n = v555 + mulq(ZS_V555 - v555, ZS_K555_CHG);
            if (v555_n >= ZS_V555_HI) o555 <= 1'b0;
        end else begin
            v555_n = v555 - mulq(v555, ZS_K555_DIS);
            if (v555_n <= ZS_V555_LO) o555 <= 1'b1;
        end
        v555 <= v555_n;

        // S2636 tone: 7406 inverter into a two-pole RC low-pass, then AC coupled
        p1  <= p1 + mulq((pvi ? ZS_PVI_LO : ZS_PVI_HI) - p1, pvi ? ZS_K_PVI1_LO : ZS_K_PVI1_HI);
        p2  <= p2 + mulq(p1 - p2, ZS_K_PVI2);
        php <= php + mulq(p2 - php, ZS_K_PVI_HP);
    end
end

// mix: SN76477 output + filtered PVI tone (P1 balance: ZS_PVI_GAIN per volt)
wire signed [31:0] pdiff = p2 - php;
wire signed [33:0] mix   = 34'(sn_out) + 34'(mulq(pdiff, ZS_PVI_GAIN));
assign out = mix > 34'sd32767 ? 16'sd32767 : mix < -34'sd32768 ? -16'sd32768 : 16'(mix);

endmodule
