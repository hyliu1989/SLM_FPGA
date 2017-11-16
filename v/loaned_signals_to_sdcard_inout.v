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
	inout        bSDCARD_CTRL_CMD,
	inout  [3:0] bSDCARD_CTRL_DATA,
	
	input        iIS_CMD_OUTPUT,
	input        iIS_DATA0_OUTPUT,
	input        iIS_DATA3_OUTPUT
);

// Clock signal
//iLOANED_CLK  // ignored
assign oLOANED_CLK_EN = 1'b1;
assign oLOANED_CLK = iSDCARD_CTRL_CLK;

// Command signal
assign oLOANED_CMD_EN = iIS_CMD_OUTPUT;
tri_state_binding tri_cmd(iLOANED_CMD, oLOANED_CMD, bSDCARD_CTRL_CMD, iIS_CMD_OUTPUT);

// Data0 signal
assign oLOANED_DATA_EN[0] = iIS_DATA0_OUTPUT;
tri_state_binding tri_data0(iLOANED_DATA[0], oLOANED_DATA[0], bSDCARD_CTRL_DATA[0], iIS_DATA0_OUTPUT);

// Data1 signal
assign oLOANED_DATA_EN[1] = 1'b0;
tri_state_binding tri_data1(iLOANED_DATA[1], oLOANED_DATA[1], bSDCARD_CTRL_DATA[1], 1'b0);

// Data2 signal
assign oLOANED_DATA_EN[2] = 1'b0;
tri_state_binding tri_data2(iLOANED_DATA[2], oLOANED_DATA[2], bSDCARD_CTRL_DATA[2], 1'b0);

// Data3 signal
assign oLOANED_DATA_EN[3] = iIS_DATA3_OUTPUT;
tri_state_binding tri_data3(iLOANED_DATA[3], oLOANED_DATA[3], bSDCARD_CTRL_DATA[3], iIS_DATA3_OUTPUT);

endmodule



module tri_state_binding(
	// one end
	input  iSIGNAL,
	output oSIGNAL,
	
	// the other end
	inout  bSIGNAL,
	
	input  iDOES_B_OUTPUT_TO_O
);
assign oSIGNAL = bSIGNAL;
assign bSIGNAL = iDOES_B_OUTPUT_TO_O? 1'bz :    // if iDOES_B_OUTPUT_TO_O==1,   don't drive bSIGNAL since bSIGNAL contains signal now
                                      iSIGNAL;  // else,                        bSIGNAL takes input from iSIGNAL

endmodule
