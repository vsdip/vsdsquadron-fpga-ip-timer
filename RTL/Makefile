# ===============================
# basicRISCV Makefile
# Simulation + FPGA build (ICE40UP5K SG48)
# ===============================

# -------- Project variables --------
TOP        := SOC
PCF_FILE   := VSDSquadronFM.pcf
FPGA_PART  := up5k
FPGA_PKG   := sg48
BOARD_FREQ := 12

# Only top RTL + external IPs
# clockworks.v and emitter_uart.v are INCLUDED inside riscv.v
VERILOG_SRCS := \
	riscv.v \
	timer_ip.v

# -------- UART variables (FPGA only) --------
PICO_DEVICE := /dev/ttyUSB0
BAUDS       := 9600

# -------- Default target --------
all: build

# -------- Simulation --------
sim:
	iverilog -g2012 -DBENCH $(VERILOG_SRCS) -o sim.out
	vvp sim.out
	gtkwave soc.vcd

# -------- FPGA Synthesis --------
synth:
	yosys -p "read_verilog $(VERILOG_SRCS); synth_ice40 -top $(TOP) -json $(TOP).json"

# -------- Place & Route --------
pnr:
	nextpnr-ice40 \
		--$(FPGA_PART) \
		--package $(FPGA_PKG) \
		--pcf $(PCF_FILE) \
		--pcf-allow-unconstrained \
		--json $(TOP).json \
		--asc $(TOP).asc

# -------- Bitstream --------
bitstream:
	icepack $(TOP).asc $(TOP).bin

# -------- Full FPGA build --------
build: synth pnr bitstream
	@echo "FPGA build complete: $(TOP).bin"

# -------- Flash FPGA --------
flash:
	sudo iceprog $(TOP).bin

# -------- UART terminal --------
terminal:
	sudo picocom -b $(BAUDS) $(PICO_DEVICE) \
	--imap lfcrlf,crcrlf --omap delbs,crlf

# -------- Clean --------
clean:
	rm -f sim.out *.vcd \
	      $(TOP).json $(TOP).asc $(TOP).bin

