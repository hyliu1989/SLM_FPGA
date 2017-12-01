	reader_system u0 (
		.clk_clk                                   (<connected-to-clk_clk>),                                   //                           clk.clk
		.jtag_uart_0_avalon_jtag_slave_chipselect  (<connected-to-jtag_uart_0_avalon_jtag_slave_chipselect>),  // jtag_uart_0_avalon_jtag_slave.chipselect
		.jtag_uart_0_avalon_jtag_slave_address     (<connected-to-jtag_uart_0_avalon_jtag_slave_address>),     //                              .address
		.jtag_uart_0_avalon_jtag_slave_read_n      (<connected-to-jtag_uart_0_avalon_jtag_slave_read_n>),      //                              .read_n
		.jtag_uart_0_avalon_jtag_slave_readdata    (<connected-to-jtag_uart_0_avalon_jtag_slave_readdata>),    //                              .readdata
		.jtag_uart_0_avalon_jtag_slave_write_n     (<connected-to-jtag_uart_0_avalon_jtag_slave_write_n>),     //                              .write_n
		.jtag_uart_0_avalon_jtag_slave_writedata   (<connected-to-jtag_uart_0_avalon_jtag_slave_writedata>),   //                              .writedata
		.jtag_uart_0_avalon_jtag_slave_waitrequest (<connected-to-jtag_uart_0_avalon_jtag_slave_waitrequest>), //                              .waitrequest
		.reset_reset_n                             (<connected-to-reset_reset_n>),                             //                         reset.reset_n
		.sdram_controller_0_s1_address             (<connected-to-sdram_controller_0_s1_address>),             //         sdram_controller_0_s1.address
		.sdram_controller_0_s1_byteenable_n        (<connected-to-sdram_controller_0_s1_byteenable_n>),        //                              .byteenable_n
		.sdram_controller_0_s1_chipselect          (<connected-to-sdram_controller_0_s1_chipselect>),          //                              .chipselect
		.sdram_controller_0_s1_writedata           (<connected-to-sdram_controller_0_s1_writedata>),           //                              .writedata
		.sdram_controller_0_s1_read_n              (<connected-to-sdram_controller_0_s1_read_n>),              //                              .read_n
		.sdram_controller_0_s1_write_n             (<connected-to-sdram_controller_0_s1_write_n>),             //                              .write_n
		.sdram_controller_0_s1_readdata            (<connected-to-sdram_controller_0_s1_readdata>),            //                              .readdata
		.sdram_controller_0_s1_readdatavalid       (<connected-to-sdram_controller_0_s1_readdatavalid>),       //                              .readdatavalid
		.sdram_controller_0_s1_waitrequest         (<connected-to-sdram_controller_0_s1_waitrequest>),         //                              .waitrequest
		.sdram_controller_0_s1_clock_clk           (<connected-to-sdram_controller_0_s1_clock_clk>),           //   sdram_controller_0_s1_clock.clk
		.sdram_controller_0_wire_addr              (<connected-to-sdram_controller_0_wire_addr>),              //       sdram_controller_0_wire.addr
		.sdram_controller_0_wire_ba                (<connected-to-sdram_controller_0_wire_ba>),                //                              .ba
		.sdram_controller_0_wire_cas_n             (<connected-to-sdram_controller_0_wire_cas_n>),             //                              .cas_n
		.sdram_controller_0_wire_cke               (<connected-to-sdram_controller_0_wire_cke>),               //                              .cke
		.sdram_controller_0_wire_cs_n              (<connected-to-sdram_controller_0_wire_cs_n>),              //                              .cs_n
		.sdram_controller_0_wire_dq                (<connected-to-sdram_controller_0_wire_dq>),                //                              .dq
		.sdram_controller_0_wire_dqm               (<connected-to-sdram_controller_0_wire_dqm>),               //                              .dqm
		.sdram_controller_0_wire_ras_n             (<connected-to-sdram_controller_0_wire_ras_n>),             //                              .ras_n
		.sdram_controller_0_wire_we_n              (<connected-to-sdram_controller_0_wire_we_n>),              //                              .we_n
		.sdram_controller_0_wire_clk_clk           (<connected-to-sdram_controller_0_wire_clk_clk>)            //   sdram_controller_0_wire_clk.clk
	);

