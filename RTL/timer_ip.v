`timescale 1ns / 1ps

module timer_ip (
    input  wire        clk,
    input  wire        resetn,

    // Bus interface
    input  wire        sel,        // timer selected
    input  wire        wr_en,       // write enable
    input  wire        rd_en,       // read enable
    input  wire [1:0]  addr,       // register select
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    // Hardware output
    output wire        timeout_o
);

    // -------------------------------------------------
    // Registers
    // -------------------------------------------------
    reg [31:0] ctrl_reg;    // CTRL
    reg [31:0] load_reg;    // LOAD
    reg [31:0] value_reg;   // VALUE
    reg        timeout;     // STATUS[0]

    // Prescaler
    reg [15:0] presc_cnt;

    // CTRL fields
    wire en        = ctrl_reg[0];
    wire mode      = ctrl_reg[1];   // 0 = one-shot, 1 = periodic
    wire presc_en  = ctrl_reg[2];
    wire [7:0] presc_div = ctrl_reg[15:8];

    // -------------------------------------------------
    // WRITE LOGIC (CTRL & LOAD only)
    // -------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            ctrl_reg <= 32'b0;
            load_reg <= 32'b0;
        end else if (sel && wr_en) begin
            case (addr)
                2'b00: ctrl_reg <= wdata;   // CTRL
                2'b01: load_reg <= wdata;   // LOAD
                default: ;
            endcase
        end
    end

    // -------------------------------------------------
    // TIMER CORE LOGIC (VALUE + TIMEOUT)
    // -------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            value_reg <= 32'b0;
            presc_cnt <= 16'b0;
            timeout   <= 1'b0;

        end else begin
            // STATUS W1C clear
            if (sel && wr_en && addr == 2'b11 && wdata[0]) begin
                timeout <= 1'b0;
            end

            if (en) begin
                // Prescaler tick
                if (!presc_en || presc_cnt == presc_div) begin
                    presc_cnt <= 16'b0;

                    if (value_reg > 1) begin
                        value_reg <= value_reg - 1;

                    end else if (value_reg == 1) begin
                        timeout <= 1'b1;

                        if (mode) begin
                            value_reg <= load_reg;   // periodic reload
                        end else begin
                            value_reg <= 32'b0;      // one-shot stops
                        end

                    end else begin
                        // value_reg == 0
                        if (mode) begin
                            value_reg <= load_reg;   // periodic keeps running
                        end else begin
                            value_reg <= 32'b0;      // one-shot stays stopped
                        end
                    end

                end else begin
                    presc_cnt <= presc_cnt + 1;
                end

            end else begin
                // EN = 0 → preload
                value_reg <= load_reg;
                presc_cnt <= 16'b0;
                timeout   <= 1'b0;
            end
        end
    end

    // -------------------------------------------------
    // READ LOGIC (registered)
    // -------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            rdata <= 32'b0;
        end else if (sel && rd_en) begin
            case (addr)
                2'b00: rdata <= ctrl_reg;         // CTRL
                2'b01: rdata <= load_reg;         // LOAD
                2'b10: rdata <= value_reg;        // VALUE
                2'b11: rdata <= {31'b0, timeout}; // STATUS
                default: rdata <= 32'b0;
            endcase
        end
    end

    // -------------------------------------------------
    // Hardware output
    // -------------------------------------------------
    assign timeout_o = timeout;

endmodule

