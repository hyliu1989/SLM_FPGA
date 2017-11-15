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

`define ENABLE_HPS

module SLMCtrl(

    ///////// ADC /////////
//    output             ADC_CONVST,
//    output             ADC_DIN,
//    input              ADC_DOUT,
//    output             ADC_SCLK,

    ///////// AUD /////////
//    input              AUD_ADCDAT,
//    inout              AUD_ADCLRCK,
//    inout              AUD_BCLK,
//    output             AUD_DACDAT,
//    inout              AUD_DACLRCK,
//    output             AUD_XCK,

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
//    output             FAN_CTRL,

    ///////// FPGA /////////
//    output             FPGA_I2C_SCLK,
//    inout              FPGA_I2C_SDAT,

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
//    inout              HPS_CONV_USB_N,
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
/*    output             HPS_ENET_GTX_CLK,
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
    inout              HPS_LTC_GPIO,*/
    inout              HPS_SD_CLK,  //output             HPS_SD_CLK,
    inout              HPS_SD_CMD,
    inout       [3:0]  HPS_SD_DATA,
/*    output             HPS_SPIM_CLK,
    input              HPS_SPIM_MISO,
    output             HPS_SPIM_MOSI,
    inout              HPS_SPIM_SS,
    input              HPS_UART_RX,
    output             HPS_UART_TX,
    input              HPS_USB_CLKOUT,
    inout       [7:0]  HPS_USB_DATA,
    input              HPS_USB_DIR,
    input              HPS_USB_NXT,
    output             HPS_USB_STP,*/
`endif /*ENABLE_HPS*/

    ///////// IRDA /////////
//    input              IRDA_RXD,
//    output             IRDA_TXD,

    ///////// KEY /////////
    input       [3:0]  KEY,

    ///////// LEDR /////////
    output      [9:0]  LEDR,

    ///////// PS2 /////////
//    inout              PS2_CLK,
//    inout              PS2_CLK2,
//    inout              PS2_DAT,
//    inout              PS2_DAT2,

    ///////// SW /////////
    input       [9:0]  SW,

    ///////// TD /////////
    input              TD_CLK27,
//    input       [7:0]  TD_DATA,
//    input              TD_HS,
    output             TD_RESET_N,
//    input              TD_VS,

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
wire        vga_clock;
wire [7:0]  vga_data;
wire        vga_fifo_rclk;
wire        vga_fifo_rreq;
wire        vga_fifo_aclear;
wire        vga_load_to_fifo_req;
wire [12:0] vga_fifo_loadline_id;
wire        fifo_wclk;
wire [7:0]  fifo_wdata;
wire        fifo_wen;

wire [24:0] sdram_ctrl_addr;

wire        sdram_ctrl_clock;
wire        sdram_ctrl_wait_req;
wire [24:0] sdram_ctrl_write_addr;
wire        sdram_ctrl_write_en;
wire [15:0] sdram_ctrl_write_data;
wire        sdram_ctrl_write_done;

wire        sdram_ctrl_read_en;
wire [24:0] sdram_ctrl_read_addr;
wire [15:0] sdram_ctrl_read_data;
wire        sdram_ctrl_read_datavalid;

wire        sd_cmd;
wire        sd_clk;
wire [3:0]  sd_data;
wire [66:0] loan_in;
wire [66:0] loan_out;
wire [66:0] loan_out_en;

wire [2:0]  hps_reset_req;
wire        hps_warm_reset, hps_cold_reset, hps_debug_reset;
wire        delayed_reset, delayed_reset_1, delayed_reset_2;
wire        trigger;

// testing signals
wire        update_x_offset;
wire        update_y_offset;
reg [7:0]   test_x_offset;
reg         test_x_offset_sign;
reg [7:0]   test_y_offset;
reg         test_y_offset_sign;

//=======================================================
//  Structural coding
//=======================================================
assign sdram_ctrl_addr = sdram_ctrl_write_done? sdram_ctrl_read_addr : sdram_ctrl_write_addr;

delay_x00_ms delay_module_0(
    .iCLOCK50(CLOCK_50),
    .iTRIGGER(!KEY[0]),
    .oDELAY100(delayed_reset),
    .oDELAY200(delayed_reset_1),
    .oDELAY300(delayed_reset_2),
    .oDELAY400()
);

delay_x00_ms delay_module_1(
    .iCLOCK50(CLOCK_50),
    .iTRIGGER(!KEY[1]),
    .oDELAY100(trigger),
    .oDELAY200(),
    .oDELAY300(),
    .oDELAY400()
);

delay_x00_ms delay_module_2(
    .iCLOCK50(CLOCK_50),
    .iTRIGGER(!KEY[2]),
    .oDELAY100(update_y_offset),
    .oDELAY200(),
    .oDELAY300(),
    .oDELAY400()
);

delay_x00_ms delay_module_3(
    .iCLOCK50(CLOCK_50),
    .iTRIGGER(!KEY[3]),
    .oDELAY100(update_x_offset),
    .oDELAY200(),
    .oDELAY300(),
    .oDELAY400()
);

assign TD_RESET_N = 1'b1;
vga_pll vga_pll_0(
    .refclk(TD_CLK27),
    .rst(delayed_reset),
    .outclk_0(vga_clock)
);

vga_control vga_ctrl_0(
    // FIFO read signal
    .iData_R(vga_data),
    .iData_G(vga_data),
    .iData_B(vga_data),
    .oFIFO_RCLK(vga_fifo_rclk),
    .oFIFO_REQ(vga_fifo_rreq),

    // FIFO load signal
    .oFIFO_LOAD_REQ(vga_load_to_fifo_req),
    .oFIFO_LOAD_VLINE(vga_fifo_loadline_id),
    .oFIFO_CLEAR(vga_fifo_aclear),
    
    //    VGA Side
    .oVGA_R(VGA_R),
    .oVGA_G(VGA_G),
    .oVGA_B(VGA_B),
    .oVGA_H_SYNC(VGA_HS),
    .oVGA_V_SYNC(VGA_VS),
    .oVGA_SYNC_N(VGA_SYNC_N),
    .oVGA_BLANK_N(VGA_BLANK_N),
    .oVGA_CLK(VGA_CLK),

    //    Control Signal
    .iCLK(vga_clock),
    .iRST_N(~delayed_reset)
);

sdram_to_vgafifo sdram_to_vgafifo_0(
    .iRST(delayed_reset_2 || !sdram_ctrl_write_done),
    .iCLK(sdram_ctrl_clock),

    // control signals for current frame
    // FIXME: the 5 values here are for testing
    .iFRAME_ID(SW[5:0]),  // input [5:0]
    .iOFFSET_H_SIGN(test_x_offset_sign),  // input
    .iOFFSET_H(test_x_offset),  // input [7:0], horizontal offset, + to the right
    .iOFFSET_V_SIGN(test_y_offset_sign),  // input
    .iOFFSET_V(test_y_offset),  // input [7:0], vertial offset, + to the bottom

    // VGA signals (as a trigger to load)
    .iVGA_LINE_TO_LOAD(vga_fifo_loadline_id),
    .iVGA_LOAD_TO_FIFO_REQ(vga_load_to_fifo_req),

    // read from SDRAM
    .iWAIT_REQUEST(sdram_ctrl_wait_req),
    .oRD_EN(sdram_ctrl_read_en),
    .oRD_ADDR(sdram_ctrl_read_addr),
    .iRD_DATA(sdram_ctrl_read_data),
    .iRD_DATAVALID(sdram_ctrl_read_datavalid),
    
    // write to FIFO
    .oFIFO_WCLK(fifo_wclk),
    .oFIFO_WDATA(fifo_wdata),
    .oFIFO_WEN(fifo_wen)
    //,.o_tests(LEDR[7:0])
);

fifo_vga fv0(
    .aclr(vga_fifo_aclear),
    .data(fifo_wdata),  // [7:0]
    .rdclk(vga_fifo_rclk),
    .rdreq(vga_fifo_rreq),
    .wrclk(fifo_wclk),
    .wrreq(fifo_wen),
    .q(vga_data),  // output [7:0]
    .rdempty(/*LEDR[9]*/),  //output
    .wrfull(/*LEDR[8]*/)  // output
);


reader_system reader_system_0(
    // clock and reset
    .clk_clk(CLOCK_50),
    .reset_reset_n(~delayed_reset),
    
    // SD card controller signals
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_chipselect(),  //  input  wire
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_address(),     //  input  wire [7:0]
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_read(),        //  input  wire
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_write(),       //  input  wire
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_byteenable(),  //  input  wire [3:0]
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_writedata(),   //  input  wire [31:0]
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_readdata(),    //  output wire [31:0]
    .altera_up_sd_card_avalon_interface_0_avalon_sdcard_slave_waitrequest(), //  output wire
    
    // SD card device wires
    .altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_cmd(sd_cmd),      //  inout  wire
    .altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat(sd_data[0]),  //  inout  wire
    .altera_up_sd_card_avalon_interface_0_conduit_end_b_SD_dat3(sd_data[3]), //  inout  wire
    .altera_up_sd_card_avalon_interface_0_conduit_end_o_SD_clock(sd_clk),    //  output  wire
    
    // SDRAM controller signals
    .sdram_controller_0_s1_address(sdram_ctrl_addr),                         //  input  wire [24:0]
    .sdram_controller_0_s1_byteenable_n(2'b00),                              //  input  wire [1:0]
    .sdram_controller_0_s1_chipselect(1'b1),                                 //  input  wire
    .sdram_controller_0_s1_writedata(sdram_ctrl_write_data),                 //  input  wire [15:0]
    .sdram_controller_0_s1_read_n(~(sdram_ctrl_read_en&&sdram_ctrl_write_done)),  //  input  wire
    .sdram_controller_0_s1_write_n(~sdram_ctrl_write_en),                    //  input  wire
    .sdram_controller_0_s1_readdata(sdram_ctrl_read_data),                   //  output wire [15:0]
    .sdram_controller_0_s1_readdatavalid(sdram_ctrl_read_datavalid),         //  output wire
    .sdram_controller_0_s1_waitrequest(sdram_ctrl_wait_req),                 //  output wire
    .sdram_controller_clock_0_clk(sdram_ctrl_clock),                         //  output wire, the clock signal that drives the controller
    
    // SDRAM device wires
    .sdram_controller_0_wire_addr(DRAM_ADDR),
    .sdram_controller_0_wire_ba(DRAM_BA),
    .sdram_controller_0_wire_cas_n(DRAM_CAS_N),
    .sdram_controller_0_wire_cke(DRAM_CKE),
    .sdram_controller_0_wire_cs_n(DRAM_CS_N),
    .sdram_controller_0_wire_dq(DRAM_DQ),
    .sdram_controller_0_wire_dqm({DRAM_UDQM,DRAM_LDQM}),
    .sdram_controller_0_wire_ras_n(DRAM_RAS_N),
    .sdram_controller_0_wire_we_n(DRAM_WE_N),
    .sys_sdram_pll_0_sdram_clk_clk(DRAM_CLK),


    // HPS part
    .hps_0_h2f_loan_io_in(loan_in),                      //  output wire [66:0]
    .hps_0_h2f_loan_io_out(loan_out),                    //  input  wire [66:0]
    .hps_0_h2f_loan_io_oe(loan_out_en),                  //  input  wire [66:0]
//    .hps_0_h2f_reset_reset_n(),                        //  output wire
    .hps_io_hps_io_gpio_inst_LOANIO36(HPS_SD_CMD),       //  inout  wire
    .hps_io_hps_io_gpio_inst_LOANIO38(HPS_SD_DATA[0]),   //  inout  wire
    .hps_io_hps_io_gpio_inst_LOANIO39(HPS_SD_DATA[1]),   //  inout  wire
    .hps_io_hps_io_gpio_inst_LOANIO45(HPS_SD_CLK),       //  inout  wire
    .hps_io_hps_io_gpio_inst_LOANIO46(HPS_SD_DATA[2]),   //  inout  wire
    .hps_io_hps_io_gpio_inst_LOANIO47(HPS_SD_DATA[3]),   //  inout  wire
    .memory_mem_a(HPS_DDR3_ADDR),                        //  output wire [14:0]
    .memory_mem_ba(HPS_DDR3_BA),                         //  output wire [2:0]
    .memory_mem_ck(HPS_DDR3_CK_P),                       //  output wire
    .memory_mem_ck_n(HPS_DDR3_CK_N),                     //  output wire
    .memory_mem_cke(HPS_DDR3_CKE),                       //  output wire
    .memory_mem_cs_n(HPS_DDR3_CS_N),                     //  output wire
    .memory_mem_ras_n(HPS_DDR3_RAS_N),                   //  output wire
    .memory_mem_cas_n(HPS_DDR3_CAS_N),                   //  output wire
    .memory_mem_we_n(HPS_DDR3_WE_N),                     //  output wire
    .memory_mem_reset_n(HPS_DDR3_RESET_N),               //  output wire
    .memory_mem_dq(HPS_DDR3_DQ),                         //  inout  wire [31:0]
    .memory_mem_dqs(HPS_DDR3_DQS_P),                     //  inout  wire [3:0]
    .memory_mem_dqs_n(HPS_DDR3_DQS_N),                   //  inout  wire [3:0]
    .memory_mem_odt(HPS_DDR3_ODT),                       //  output wire
    .memory_mem_dm(HPS_DDR3_DM),                         //  output wire [3:0]
    .memory_oct_rzqin(HPS_DDR3_RZQ),                     //  input  wire

//    .in_system_sources_probes_0_hps_reset_source(hps_reset_req),             //  output wire
//    .hps_0_f2h_cold_reset_req_reset_n(hps_cold_reset),                       //  input  wire
//    .hps_0_f2h_debug_reset_req_reset_n(hps_debug_reset),                     //  input  wire
//    .hps_0_f2h_warm_reset_req_reset_n(hps_warm_reset)                        //  input  wire
);

loaned_signals_to_sdcard_inout loaned_wiring_0(
    .iLOANED_CLK(loan_in[45]),
    .iLOANED_CMD(loan_in[36]),
    .iLOANED_DATA({loan_in[47:46],loan_in[39:38]}),
    
    .oLOANED_CLK(loan_out[45]),                        // output to the HPS
    .oLOANED_CMD(loan_out[36]),                        // output to the HPS
    .oLOANED_DATA({loan_out[47:46],loan_out[39:38]}),  // output to the HPS
    .oLOANED_CLK_EN(loan_out_en[45]),                  // output to the HPS
    .oLOANED_CMD_EN(loan_out_en[36]),                  // output to the HPS
    .oLOANED_DATA_EN({loan_out_en[47:46],loan_out_en[39:38]}), // output to the HPS
    
    .iSDCARD_CTRL_CLK(sd_clk),
    .ioSDCARD_CTRL_CMD(sd_cmd),
    .ioSDCARD_CTRL_DATA(sd_data)
);
// synthesize out unused pins
assign loan_out_en[66:48] = 0;
assign loan_out_en[44:40] = 0;
assign loan_out_en[37] = 0;
assign loan_out_en[35:0] = 0;
assign loan_out[66:48] = 0;
assign loan_out[44:40] = 0;
assign loan_out[37] = 0;
assign loan_out[35:0] = 0;


altera_edge_detector pulse_cold_reset (
  .clk       (CLOCK_50),
  .rst_n     (~delayed_reset),
  .signal_in (hps_reset_req[0]),
  .pulse_out (hps_cold_reset)
);
  defparam pulse_cold_reset.PULSE_EXT = 6;
  defparam pulse_cold_reset.EDGE_TYPE = 1;
  defparam pulse_cold_reset.IGNORE_RST_WHILE_BUSY = 1;

altera_edge_detector pulse_warm_reset (
  .clk       (CLOCK_50),
  .rst_n     (~delayed_reset),
  .signal_in (hps_reset_req[1]),
  .pulse_out (hps_warm_reset)
);
  defparam pulse_warm_reset.PULSE_EXT = 2;
  defparam pulse_warm_reset.EDGE_TYPE = 1;
  defparam pulse_warm_reset.IGNORE_RST_WHILE_BUSY = 1;
  
altera_edge_detector pulse_debug_reset (
  .clk       (CLOCK_50),
  .rst_n     (~delayed_reset),
  .signal_in (hps_reset_req[2]),
  .pulse_out (hps_debug_reset)
);
  defparam pulse_debug_reset.PULSE_EXT = 32;
  defparam pulse_debug_reset.EDGE_TYPE = 1;
  defparam pulse_debug_reset.IGNORE_RST_WHILE_BUSY = 1;

// testing code for sdram writing
test_sdram_write test_sdram_write_0(
    .iCLK(sdram_ctrl_clock),
    .iRST(delayed_reset_1),

    .iTRIGGER(trigger),
    .iWAIT_REQUEST(sdram_ctrl_wait_req),
    .oWR_EN(sdram_ctrl_write_en),
    .oWR_DATA(sdram_ctrl_write_data),
    .oWR_ADDR(sdram_ctrl_write_addr),
    .oDONE(sdram_ctrl_write_done)
);

assign HEX5 = sdram_ctrl_write_done? 7'b1111111 : 7'b0000011;  // letter b
assign HEX4 = sdram_ctrl_write_done? 7'b1111111 : 7'b1000001;  // letter u
assign HEX3 = sdram_ctrl_write_done? 7'b1111111 : 7'b0010010;  // letter s
assign HEX2 = sdram_ctrl_write_done? 7'b1111111 : 7'b0010001;  // letter y
assign HEX1 = 7'h7f;
assign HEX0 = 7'h7f;

always @ (posedge CLOCK_50 or posedge delayed_reset) begin
    if(delayed_reset) begin
        test_x_offset = 127;
        test_x_offset_sign = 1;
        test_y_offset = 3;
        test_y_offset_sign = 0;
    end
    else begin
        if(update_x_offset) begin
            test_x_offset = SW[7:0];
            test_x_offset_sign = SW[9];
        end
        if(update_y_offset) begin
            test_y_offset = SW[7:0];
            test_y_offset_sign = SW[9];
        end
    end
end

// // testing code for sdram reading
// test_sdram_read test_sdram_read_0(
//  .iCLK(sdram_ctrl_clock),
//  .iRST(delayed_reset_1),
//  
//  .iTEST_WRITE_DONE(sdram_ctrl_write_done),
//  
//  .iWAIT_REQUEST(sdram_ctrl_wait_req),
//  .oRD_EN(sdram_ctrl_read_en),
//  .oRD_ADDR(sdram_ctrl_read_addr),
//  .iRD_DATA(sdram_ctrl_read_data),
//  .iRD_DATAVALID(sdram_ctrl_read_datavalid),
//  .oDATA(LEDR[7:0]),
//  
//  .iSW(SW),
//  .iUPDATE_FRAME(update_frame_to_read),  // KEY 3
//  .iUPDATE_Y(update_y_to_read)  // KEY 2
// );

endmodule

