//============================================================================
//
//  chip_pool_pkg — bus types for a multi-board core's shared CPU / sound chip pool (canonical; same file in every core)
//  Copyright (C) 2026 Rodimus
//
//  Only one board runs at a time, so the CPUs and sound chips are instanced once in rtl/konami82_board.sv
//  and the active board drives them through these buses (selected by variant). *_i = board to chip,
//  *_o = chip to every board. Reset is the core reset.
//
//============================================================================

package chip_pool_pkg;

// Z80 (T80s, Mode 0, T2Write 1, IOWait 1; WAIT_n / BUSRQ_n inactive)
typedef struct packed {
    logic        ce;
    logic        int_n;
    logic        nmi_n;
    logic  [7:0] di;
} z80_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        m1_n;
    logic        mreq_n;
    logic        iorq_n;
    logic        rd_n;
    logic        wr_n;
    logic        rfsh_n;
} z80_o_t;

// MC6809E (mc6809is; HALT / DMABREQ inactive)
typedef struct packed {
    logic        fall_e;
    logic        fall_q;
    logic        irq_n;
    logic        firq_n;
    logic        nmi_n;
    logic  [7:0] d;
} m6809_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        rw;
    logic        lic;
} m6809_o_t;

// AY-3-8910 (jt49_bus, sel = 1)
typedef struct packed {
    logic        ce;
    logic        bdir;
    logic        bc1;
    logic  [7:0] din;
    logic  [7:0] ioa_in;
    logic  [7:0] iob_in;
} ay_i_t;

typedef struct packed {
    logic  [7:0] dout;
    logic  [7:0] a;
    logic  [7:0] b;
    logic  [7:0] c;
    logic  [7:0] iob_out;
} ay_o_t;

// 8039 (T48 t8039_notri_extrom, EA low, program on pmem; T0 / T1 low, P1 / P2 inputs high)
typedef struct packed {
    logic        xtal_en;
    logic        int_n;
    logic  [7:0] db;
    logic  [7:0] pmem_d;
} mcs48_i_t;

typedef struct packed {
    logic  [7:0] p1;
    logic  [7:0] p2;
    logic [11:0] pmem_a;
    logic        rd_n;
} mcs48_o_t;

// MC6800 / M6802 (jt680x, 6802 microcode; ext_halt and the 6801 on-chip interrupts inactive). rst is on the bus:
// Space Invaders' 8080 board resets it with its watchdog and holds it in reset on sets that do not use it
typedef struct packed {
    logic        rst;
    logic        cen;
    logic        irq;
    logic        nmi;
    logic  [7:0] din;
} m6800_i_t;

typedef struct packed {
    logic [15:0] a;
    logic  [7:0] dout;
    logic        wr;
} m6800_o_t;

endpackage
