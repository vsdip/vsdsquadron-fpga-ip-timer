# Timer IP – Integration Guide (VSDSquadron SoC)

This guide explains how to integrate the **Timer IP** into a **VSDSquadron RISC-V SoC** design. 

---

## Required RTL Files

Copy the following RTL file into your SoC RTL project:

<details>
<summary> Timer IP (click to expand) </summary>

```verilog

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
            if (sel && wr_en && addr == 2'b11 && wdata[0])
                timeout <= 1'b0;

            if (en) begin
                // Prescaler tick
                if (!presc_en || presc_cnt == presc_div) begin
                    presc_cnt <= 16'b0;

                    if (value_reg > 1) begin
                        value_reg <= value_reg - 1;

                    end else if (value_reg == 1) begin
                        timeout <= 1'b1;
                        value_reg <= mode ? load_reg : 32'b0;

                    end else begin
                        // value_reg == 0
                        value_reg <= load_reg;
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
                2'b00: rdata <= ctrl_reg;              // CTRL
                2'b01: rdata <= load_reg;              // LOAD
                2'b10: rdata <= value_reg;             // VALUE
                2'b11: rdata <= {31'b0, timeout};      // STATUS
                default: rdata <= 32'b0;
            endcase
        end
    end

    // -------------------------------------------------
    // Hardware output
    // -------------------------------------------------
    assign timeout_o = timeout;

endmodule

```

</details>

---

## Where to Instantiate the IP

Instantiate the Timer IP inside the **SoC top-level module** where other **memory-mapped peripherals** (GPIO, UART, SPI) are connected.

The Timer IP must be instantiated in the **IO page section** of the SoC.

---

### Timer IP Instantiation Template

```verilog
wire [31:0] timer_rdata;
wire        timer_timeout;

timer_ip TIMER (
    .clk      (clk),
    .resetn   (resetn),

    // Bus interface
    .sel      (timer_sel),
    .wr_en    (timer_wr_en),
    .rd_en    (timer_rd_en),
    .addr     (timer_addr),
    .wdata    (mem_wdata),
    .rdata    (timer_rdata),

    // Hardware output
    .timeout_o(timer_timeout)
);
```

---

## Defining New Timer Signals in SoC Module

```verilog
wire        timer_sel;
wire        timer_wr_en;
wire        timer_rd_en;
wire [1:0]  timer_addr;
wire [31:0] timer_rdata;
wire        timer_timeout;
```

---

## Address Decoding Expectations

### IO Page Selection

```verilog
isIO = mem_addr[22];
```

### Timer Select Logic

```verilog
localparam TIMER_BASE_WADDR = 30'h00100010; // 0x00400040 >> 2

assign timer_sel =
    isIO &&
    (mem_wordaddr >= TIMER_BASE_WADDR) &&
    (mem_wordaddr <= TIMER_BASE_WADDR + 3);
```

### Read / Write Enables

```verilog
assign timer_wr_en = timer_sel && (|mem_wmask);
assign timer_rd_en = timer_sel && mem_rstrb;
```

### Register Offset Selection

```verilog
assign timer_addr = mem_wordaddr[1:0];
```

---

## Software Base Address

```text
TIMER_BASE = 0x00400040
```

---

## IO Read Data Mux Update

```verilog
wire [31:0] IO_rdata =
    timer_sel ? timer_rdata :
    mem_wordaddr[IO_UART_CNTL_bit] ? {22'b0, !uart_ready, 9'b0} :
    mem_wordaddr[IO_gpio_bit] ? gpio_rdata :
    32'b0;

assign mem_rdata = isRAM ? RAM_rdata : IO_rdata;
```

---

## Signals Exposed to SoC Top-Level

| Signal | Direction | Description |
|------|----------|-------------|
| timeout_o | output | Timer expiration pulse |

---

## Board-Level Connections (VSDSquadron FPGA)

### Board-Level Signal Mapping

| Timer IP Signal | SoC Signal | FPGA Pin | Board Connection | Purpose |
|---------------|-----------|---------|-----------------|---------|
| timeout_o | LEDS[0] | 39 | On-board LED0 | Visual timeout indication |


---

### Example Constraint File Entry

```pcf
set_io LEDS[0] 39
```

---


