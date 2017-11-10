module	sdram_to_vga_fifo(
	input         iRST,
	input         iCLK,
	
	// control signals for current frame
	input [5:0]   iFRAME_ID,
	input         iOFFSET_H_SIGN,
	input [8:0]   iOFFSET_H,  // horizontal offset, + to the right
	input         iOFFSET_V_SIGN,
	input [8:0]   iOFFSET_V,  // vertial offset, + to the bottom
	
	// VGA signals (as a trigger to load)
	input [12:0]  iVGA_LINE_TO_LOAD,
	input         iVGA_LOAD_TO_FIFO_REQ,
	
	// read from SDRAM
	input         iWAIT_REQUEST,
	output        oRD_EN,
	output [24:0] oRD_ADDR,
	input  [15:0] iRD_DATA,
	input         iRD_DATAVALID,
	output [7:0]  oDATA,
	
	// write to FIFO
	output        oWCLK,
	output [7:0]  oWDATA,
	output        oWEN,
);


reg [7:0]	data;
reg [3:0]	state;
reg [12:0]	horizontal_counter;

assign oWCLK = ~iCLK;
assign oWEN = (state == 4'd2);
assign oWDATA = data;




always @ (*) begin
	if (iVGA_LINE_TO_LOAD[10:0] >= {1'b0,test_signal_0} && iVGA_LINE_TO_LOAD[10:0] < {1'b0,test_signal_0} + 11'd100) begin
		if (horizontal_counter < (iVGA_LINE_TO_LOAD[10:0] - {1'b0,test_signal_0}))
			data = 8'h00;
		else
			data = 8'hff;
	end
	else begin
		data = 8'h00;
	end
end

always @ (posedge iCLK or posedge iRST) begin
	if (!iRST) begin
		state <= 4'd0;
	end
	else begin
		case(state)
			4'd0: begin  // listening to the kicking
				if (iVGA_LOAD_TO_FIFO_REQ)
					state <= 4'd1;
			end
			4'd1: begin  // waiting for the kicking signal to turn off
				horizontal_counter <= 13'd0;
				if (!iVGA_LOAD_TO_FIFO_REQ)
					state <= 4'd2;
			end
			4'd2: begin  // loading the memory into the FIFO
				horizontal_counter <= horizontal_counter + 1'b1;
				if (horizontal_counter == 13'd1280-13'd1)
					state <= 4'd0;
				else
					state <= state;
			end
			default:  // error state, go back to idle(4'd0)
				state <= 4'd0;
		endcase
	end
end
// end of temporary module
endmodule
