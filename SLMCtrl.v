// ============================================================================
// Copyright (c) 2013 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//  
//  
//                     web: http://www.terasic.com/  
//                     email: support@terasic.com
//
// ============================================================================
//Date:  Thu Jul 11 11:26:45 2013
// ============================================================================

//`define ENABLE_HPS

module SLMCtrl(

      ///////// ADC /////////
      output             ADC_CONVST,
      output             ADC_DIN,
      input              ADC_DOUT,
      output             ADC_SCLK,

      ///////// AUD /////////
      input              AUD_ADCDAT,
      inout              AUD_ADCLRCK,
      inout              AUD_BCLK,
      output             AUD_DACDAT,
      inout              AUD_DACLRCK,
      output             AUD_XCK,

      ///////// CLOCK2 /////////
      input              CLOCK2_50,

      ///////// CLOCK3 /////////
      input              CLOCK3_50,

      ///////// CLOCK4 /////////
      input              CLOCK4_50,

      ///////// CLOCK /////////
      input              CLOCK_50,

      ///////// DRAM /////////
      output      [12:0] DRAM_ADDR,
      output      [1:0]  DRAM_BA,
      output             DRAM_CAS_N,
      output             DRAM_CKE,
      output             DRAM_CLK,
      output             DRAM_CS_N,
      inout       [15:0] DRAM_DQ,
      output             DRAM_LDQM,
      output             DRAM_RAS_N,
      output             DRAM_UDQM,
      output             DRAM_WE_N,

      ///////// FAN /////////
      output             FAN_CTRL,

      ///////// FPGA /////////
      output             FPGA_I2C_SCLK,
      inout              FPGA_I2C_SDAT,

      ///////// GPIO /////////
      inout     [35:0]         GPIO_0,
      inout     [35:0]         GPIO_1,
 

      ///////// HEX0 /////////
      output      [6:0]  HEX0,

      ///////// HEX1 /////////
      output      [6:0]  HEX1,

      ///////// HEX2 /////////
      output      [6:0]  HEX2,

      ///////// HEX3 /////////
      output      [6:0]  HEX3,

      ///////// HEX4 /////////
      output      [6:0]  HEX4,

      ///////// HEX5 /////////
      output      [6:0]  HEX5,

`ifdef ENABLE_HPS
      ///////// HPS /////////
      inout              HPS_CONV_USB_N,
      output      [14:0] HPS_DDR3_ADDR,
      output      [2:0]  HPS_DDR3_BA,
      output             HPS_DDR3_CAS_N,
      output             HPS_DDR3_CKE,
      output             HPS_DDR3_CK_N,
      output             HPS_DDR3_CK_P,
      output             HPS_DDR3_CS_N,
      output      [3:0]  HPS_DDR3_DM,
      inout       [31:0] HPS_DDR3_DQ,
      inout       [3:0]  HPS_DDR3_DQS_N,
      inout       [3:0]  HPS_DDR3_DQS_P,
      output             HPS_DDR3_ODT,
      output             HPS_DDR3_RAS_N,
      output             HPS_DDR3_RESET_N,
      input              HPS_DDR3_RZQ,
      output             HPS_DDR3_WE_N,
      output             HPS_ENET_GTX_CLK,
      inout              HPS_ENET_INT_N,
      output             HPS_ENET_MDC,
      inout              HPS_ENET_MDIO,
      input              HPS_ENET_RX_CLK,
      input       [3:0]  HPS_ENET_RX_DATA,
      input              HPS_ENET_RX_DV,
      output      [3:0]  HPS_ENET_TX_DATA,
      output             HPS_ENET_TX_EN,
      inout       [3:0]  HPS_FLASH_DATA,
      output             HPS_FLASH_DCLK,
      output             HPS_FLASH_NCSO,
      inout              HPS_GSENSOR_INT,
      inout              HPS_I2C1_SCLK,
      inout              HPS_I2C1_SDAT,
      inout              HPS_I2C2_SCLK,
      inout              HPS_I2C2_SDAT,
      inout              HPS_I2C_CONTROL,
      inout              HPS_KEY,
      inout              HPS_LED,
      inout              HPS_LTC_GPIO,
      output             HPS_SD_CLK,
      inout              HPS_SD_CMD,
      inout       [3:0]  HPS_SD_DATA,
      output             HPS_SPIM_CLK,
      input              HPS_SPIM_MISO,
      output             HPS_SPIM_MOSI,
      inout              HPS_SPIM_SS,
      input              HPS_UART_RX,
      output             HPS_UART_TX,
      input              HPS_USB_CLKOUT,
      inout       [7:0]  HPS_USB_DATA,
      input              HPS_USB_DIR,
      input              HPS_USB_NXT,
      output             HPS_USB_STP,
`endif /*ENABLE_HPS*/

      ///////// IRDA /////////
      input              IRDA_RXD,
      output             IRDA_TXD,

      ///////// KEY /////////
      input       [3:0]  KEY,

      ///////// LEDR /////////
      output      [9:0]  LEDR,

      ///////// PS2 /////////
      inout              PS2_CLK,
      inout              PS2_CLK2,
      inout              PS2_DAT,
      inout              PS2_DAT2,

      ///////// SW /////////
      input       [9:0]  SW,

      ///////// TD /////////
      input              TD_CLK27,
      input      [7:0]  TD_DATA,
      input             TD_HS,
      output             TD_RESET_N,
      input             TD_VS,

      ///////// VGA /////////
      output      [7:0]  VGA_B,
      output             VGA_BLANK_N,
      output             VGA_CLK,
      output      [7:0]  VGA_G,
      output             VGA_HS,
      output      [7:0]  VGA_R,
      output             VGA_SYNC_N,
      output             VGA_VS
);


//=======================================================
//  REG/WIRE declarations
//=======================================================
wire 		vga_clock;
wire [7:0]	vga_data;
wire		vga_fifo_rclk;
wire		vga_fifo_rreq;
wire		vga_fifo_aclear;
wire		vga_fifo_loadreq;
wire [12:0]	vga_fifo_loadline;

// for delayed reset signal
reg  [26:0] local_counter;
reg			delayed_reset_n;

//=======================================================
//  Functional coding
//=======================================================
// for delayed reset signal
always @ (posedge CLOCK_50) begin
	if (local_counter != 28'd0)
		local_counter <= local_counter + 1'b1;	
	else
		local_counter <= local_counter + !KEY[0];
end
always @ (*) begin
	// 						   27'b000_0100_1100_0100_1011_0100_0000 // number of clocks for 0.1 second
	if (local_counter[26:22] == 5'b011_10)
		delayed_reset_n = 1'b0;
	else
		delayed_reset_n = 1'b1;
end
assign LEDR[0] = delayed_reset_n;

/*
reg [29:0] temp_cnt;
always @ (posedge vga_clock) begin
	temp_cnt <= temp_cnt + 1'b1;
end
assign LEDR[1] = temp_cnt[28];

reg [29:0] temp_cnt2;
always @ (posedge TD_CLK27) begin
	temp_cnt2 <= temp_cnt2 + 1'b1;
end
assign LEDR[2] = temp_cnt2[25];
*/

//=======================================================
//  Structural coding
//=======================================================
assign TD_RESET_N = 1'b1;
vga_pll vga_pll_0(
	.refclk(TD_CLK27),
	.rst(~delayed_reset_n),
	.outclk_0(vga_clock)
);

VGA_Controller vga_ctrl_0(
	// FIFO read signal
	.iData_R(vga_data),
	.iData_G(vga_data),
	.iData_B(vga_data),
	.oFIFO_RCLK(vga_fifo_rclk),
	.oFIFO_REQ(vga_fifo_rreq),
	// FIFO load signal
	.oFIFO_LOAD_REQ(vga_fifo_loadreq),
	.oFIFO_LOAD_VLINE(vga_fifo_loadline),
	.oFIFO_CLEAR(vga_fifo_aclear),
	
	//	VGA Side
	.oVGA_R(VGA_R),
	.oVGA_G(VGA_G),
	.oVGA_B(VGA_B),
	.oVGA_H_SYNC(VGA_HS),
	.oVGA_V_SYNC(VGA_VS),
	.oVGA_SYNC_N(VGA_SYNC_N),
	.oVGA_BLANK_N(VGA_BLANK_N),
	.oVGA_CLK(VGA_CLK),

	//	Control Signal
	.iCLK(vga_clock),
	.iRST_N(delayed_reset_n)
);

// Temporary testing module starts here for fifo writing (should be replaced by memory module) 
reg [7:0]	temp_mem_data;
reg [3:0]	temp_state;
reg [10:0]	temp_load_counter;
wire		temp_wclk;
wire		temp_clk;
wire		temp_reset_n;
reg			temp_wen;
assign temp_clk 		= vga_clock;
assign temp_reset_n 	= delayed_reset_n;
assign temp_wclk		= ~temp_clk;

always @ (*) begin
	if (vga_fifo_loadline[10:0] >= {1'b0,SW} && vga_fifo_loadline[10:0] < {1'b0,SW} + 11'd100)
		temp_mem_data = 8'hff;
	else
		temp_mem_data = 8'h00;
	
	temp_wen = (temp_state == 4'd2);
end

always @ (posedge temp_clk or negedge temp_reset_n) begin
	if (!temp_reset_n) begin
		temp_state <= 4'd0;
	end
	else begin
		case(temp_state)
			4'd0: begin  // listening to the kicking
				if (vga_fifo_loadreq)
					temp_state <= 4'd1;
			end
			4'd1: begin  // waiting for the kicking signal to turn off
				temp_load_counter <= 11'd0;
				if (!vga_fifo_loadreq)
					temp_state <= 4'd2;
			end
			4'd2: begin  // loading the memory into the FIFO
				temp_load_counter <= temp_load_counter + 1'b1;
				if (temp_load_counter == 11'd1280-11'd1)
					temp_state <= 4'd0;
				else
					temp_state <= temp_state;
			end
			default:  // error state, go back to idle(4'd0)
				temp_state <= 4'd0;
		endcase
	end
end
// end of temporary module

vga_fifo vf0(
	.aclr(vga_fifo_aclear),
	.data(temp_mem_data),  // [7:0]
	.rdclk(vga_fifo_rclk),
	.rdreq(vga_fifo_rreq),
	.wrclk(temp_wclk),
	.wrreq(temp_wen),
	.q(vga_data),  // output [7:0]
	.rdempty(),  //output
	.wrfull()  // output
);


endmodule
