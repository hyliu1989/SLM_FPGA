module	sdram_to_vga_fifo(
	iRST_N,
	iCLK,
	iVGA_LINE_TO_LOAD,
	iVGA_LOAD_TO_FIFO_REQ,
	
	oWCLK,
	oWDATA,
	oWEN,
	
	test_signal_0
);

// Temporary testing module starts here for fifo writing (should be replaced by memory module) 
input wire [12:0]	iVGA_LINE_TO_LOAD;
input wire			iVGA_LOAD_TO_FIFO_REQ;
input wire			iCLK;
input wire			iRST_N;
output reg			oWEN;
output reg [7:0]	oWDATA;
output wire			oWCLK;
assign oWCLK	= ~iCLK;

reg [3:0]	state;
reg [12:0]	horizontal_counter;

// test signals
input wire [9:0]	test_signal_0;  // offsetting the testing strip


always @ (*) begin
	if (iVGA_LINE_TO_LOAD[10:0] >= {1'b0,test_signal_0} && iVGA_LINE_TO_LOAD[10:0] < {1'b0,test_signal_0} + 11'd100)
		oWDATA = 8'hff;
	else
		oWDATA = 8'h00;
	
	oWEN = (state == 4'd2);
end

always @ (posedge iCLK or negedge iRST_N) begin
	if (!iRST_N) begin
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
