
module reader_system (
	clk_clk,
	jtag_uart_0_avalon_jtag_slave_chipselect,
	jtag_uart_0_avalon_jtag_slave_address,
	jtag_uart_0_avalon_jtag_slave_read_n,
	jtag_uart_0_avalon_jtag_slave_readdata,
	jtag_uart_0_avalon_jtag_slave_write_n,
	jtag_uart_0_avalon_jtag_slave_writedata,
	jtag_uart_0_avalon_jtag_slave_waitrequest,
	reset_reset_n,
	sdram_controller_0_s1_address,
	sdram_controller_0_s1_byteenable_n,
	sdram_controller_0_s1_chipselect,
	sdram_controller_0_s1_writedata,
	sdram_controller_0_s1_read_n,
	sdram_controller_0_s1_write_n,
	sdram_controller_0_s1_readdata,
	sdram_controller_0_s1_readdatavalid,
	sdram_controller_0_s1_waitrequest,
	sdram_controller_0_s1_clock_clk,
	sdram_controller_0_wire_addr,
	sdram_controller_0_wire_ba,
	sdram_controller_0_wire_cas_n,
	sdram_controller_0_wire_cke,
	sdram_controller_0_wire_cs_n,
	sdram_controller_0_wire_dq,
	sdram_controller_0_wire_dqm,
	sdram_controller_0_wire_ras_n,
	sdram_controller_0_wire_we_n,
	sdram_controller_0_wire_clk_clk);	

	input		clk_clk;
	input		jtag_uart_0_avalon_jtag_slave_chipselect;
	input		jtag_uart_0_avalon_jtag_slave_address;
	input		jtag_uart_0_avalon_jtag_slave_read_n;
	output	[31:0]	jtag_uart_0_avalon_jtag_slave_readdata;
	input		jtag_uart_0_avalon_jtag_slave_write_n;
	input	[31:0]	jtag_uart_0_avalon_jtag_slave_writedata;
	output		jtag_uart_0_avalon_jtag_slave_waitrequest;
	input		reset_reset_n;
	input	[24:0]	sdram_controller_0_s1_address;
	input	[1:0]	sdram_controller_0_s1_byteenable_n;
	input		sdram_controller_0_s1_chipselect;
	input	[15:0]	sdram_controller_0_s1_writedata;
	input		sdram_controller_0_s1_read_n;
	input		sdram_controller_0_s1_write_n;
	output	[15:0]	sdram_controller_0_s1_readdata;
	output		sdram_controller_0_s1_readdatavalid;
	output		sdram_controller_0_s1_waitrequest;
	output		sdram_controller_0_s1_clock_clk;
	output	[12:0]	sdram_controller_0_wire_addr;
	output	[1:0]	sdram_controller_0_wire_ba;
	output		sdram_controller_0_wire_cas_n;
	output		sdram_controller_0_wire_cke;
	output		sdram_controller_0_wire_cs_n;
	inout	[15:0]	sdram_controller_0_wire_dq;
	output	[1:0]	sdram_controller_0_wire_dqm;
	output		sdram_controller_0_wire_ras_n;
	output		sdram_controller_0_wire_we_n;
	output		sdram_controller_0_wire_clk_clk;
endmodule
