module loaned_signals_to_sdcard_inout(
	input        iLOANED_CLK,
	input        iLOANED_CMD,
	input  [3:0] iLOANED_DATA,
	output       oLOANED_CLK,     // output to the HPS
	output       oLOANED_CMD,     // output to the HPS
	output [3:0] oLOANED_DATA,    // output to the HPS
	output       oLOANED_CLK_EN,  // output to the HPS
	output       oLOANED_CMD_EN,  // output to the HPS
	output [3:0] oLOANED_DATA_EN, // output to the HPS
	
	input        iSDCARD_CTRL_CLK,
	inout        ioSDCARD_CTRL_CMD,
	inout  [3:0] ioSDCARD_CTRL_DATA
);

//iLOANED_CLK  // ignored
assign oLOANED_CLK_EN = 1'b1;
assign oLOANED_CLK = iSDCARD_CTRL_CLK;

// TODO

endmodule