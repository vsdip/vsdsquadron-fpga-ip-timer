// =====================================================
// iCE40 FPGA primitive stubs for simulation (Icarus)
// =====================================================

// -------- High Frequency Oscillator --------
module SB_HFOSC #(
    parameter CLKHF_DIV = "0b00"
)(
    output CLKHF,
    input  CLKHFEN,
    input  CLKHFPU
);
    reg clk = 0;
    always #5 clk = ~clk;   // simple simulation clock
    assign CLKHF = clk;
endmodule


// -------- PLL --------
module SB_PLL40_CORE #(
    parameter FEEDBACK_PATH = "SIMPLE",
    parameter PLLOUT_SELECT = "GENCLK",
    parameter DIVR = 0,
    parameter DIVF = 0,
    parameter DIVQ = 0,
    parameter FILTER_RANGE = 0
)(
    input  REFERENCECLK,
    input  RESETB,
    input  BYPASS,
    output PLLOUTCORE,
    output PLLOUTGLOBAL,
    output LOCK
);
    // For simulation, just forward the reference clock
    assign PLLOUTCORE   = REFERENCECLK;
    assign PLLOUTGLOBAL = REFERENCECLK;
    assign LOCK         = 1'b1;
endmodule

