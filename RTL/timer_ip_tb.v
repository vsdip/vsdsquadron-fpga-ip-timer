`timescale 1ns / 1ps

module tb_timer_ip;

    // ----------------------------------
    // Clock / Reset
    // ----------------------------------
    reg clk;
    reg resetn;

    initial clk = 0;
    always #5 clk = ~clk;   // 100 MHz clock

    // ----------------------------------
    // Bus signals
    // ----------------------------------
    reg        sel;
    reg        wr_en;
    reg        rd_en;
    reg [1:0]  addr;
    reg [31:0] wdata;
    wire [31:0] rdata;

    // ----------------------------------
    // DUT
    // ----------------------------------
    timer_ip DUT (
        .clk   (clk),
        .resetn(resetn),
        .sel   (sel),
        .wr_en (wr_en),
        .rd_en (rd_en),
        .addr  (addr),
        .wdata (wdata),
        .rdata (rdata)
    );

    // ----------------------------------
    // Tasks
    // ----------------------------------
    task write_reg(input [1:0] a, input [31:0] d);
    begin
        @(posedge clk);
        sel   <= 1;
        wr_en <= 1;
        rd_en <= 0;
        addr  <= a;
        wdata <= d;
        @(posedge clk);
        sel   <= 0;
        wr_en <= 0;
    end
    endtask

    task read_reg(input [1:0] a);
    begin
        @(posedge clk);
        sel   <= 1;
        wr_en <= 0;
        rd_en <= 1;
        addr  <= a;
        @(posedge clk);
        sel   <= 0;
        rd_en <= 0;
    end
    endtask

    // ----------------------------------
    // Test Sequence
    // ----------------------------------
    initial begin
        // Default values
        sel   = 0;
        wr_en = 0;
        rd_en = 0;
        addr  = 0;
        wdata = 0;

        // Reset
        resetn = 0;
        repeat (3) @(posedge clk);
        resetn = 1;

        // ----------------------------------
        // TEST 1: One-shot timer
        // ----------------------------------
        $display("\n--- One-shot Timer Test ---");

        write_reg(2'b01, 32'd10);        // LOAD = 10
        write_reg(2'b00, 32'b0001);      // CTRL: EN=1, one-shot

        repeat (15) @(posedge clk);

        read_reg(2'b11);                 // STATUS
        $display("STATUS (expect 1): %h", rdata);

        // Clear timeout
        write_reg(2'b11, 32'h1);

        read_reg(2'b11);
        $display("STATUS after clear (expect 0): %h", rdata);

        // ----------------------------------
        // TEST 2: Periodic timer
        // ----------------------------------
        $display("\n--- Periodic Timer Test ---");

        write_reg(2'b01, 32'd10) ;    // LOAD = 5
        write_reg(2'b00, 32'b0011);       // CTRL: EN=1, MODE=1 (periodic)

        repeat (20) begin
            @(posedge clk);
            read_reg(2'b10);             // VALUE
            $display("VALUE = %d", rdata);
        end

        read_reg(2'b11);                 // STATUS
        $display("STATUS (expect 1): %h", rdata);

        $display("\n--- TEST COMPLETE ---");
        $finish;
    end

endmodule
