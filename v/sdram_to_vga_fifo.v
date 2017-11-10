module	sdram_to_vga_fifo(
	input         iRST,
	input         iCLK,
	
	// control signals for current frame
	input [5:0]   iFRAME_ID,
	input         iOFFSET_H_SIGN,
	input [7:0]   iOFFSET_H,  // horizontal offset, + to the right
	input         iOFFSET_V_SIGN,
	input [7:0]   iOFFSET_V,  // vertial offset, + to the bottom
	
	// VGA signals (as a trigger to load)
	input [12:0]  iVGA_LINE_TO_LOAD,
	input         iVGA_LOAD_TO_FIFO_REQ,
	
	// read from SDRAM
	input         iWAIT_REQUEST,
	output        oRD_EN,
	output [24:0] oRD_ADDR,
	input  [15:0] iRD_DATA,
	input         iRD_DATAVALID,
	
	// write to FIFO
	output        oFIFO_WCLK,
	output [7:0]  oFIFO_WDATA,
	output        oFIFO_WEN
);




// =================================
//  Clocks for submodules
// =================================
wire       clock_fifowriter;
wire       clock;
fifo_pll fifo_pll_0(
	.refclk(iCLK),   //  refclk.clk
	.rst(iRST),      //   reset.reset
	.outclk_0(clock_fifowriter)  // outclk0.clk
	);
assign clock = iCLK;
// TODO: use pll. clock_fifowriter should be twice as fast as the sdram iCLK.
//                clock should be the same as iCLK.
//                use pll to generate two to make sure they are in phase



// =================================
//  FIFO writer submodule
// =================================
/* A simple module to achieve writing 8-bit data into fifo from a 16-bit data
                      __    __    __    __    __    __
clock_fifowriter   __|  |__|  |__|  |__|  |__|  |__|  |__
                   _____       _____       _____       __
clock                   |_____|     |_____|     |_____|
                         ___________ ___________
r_data_valid       _____|           |           |________ (change at negedge of clock if all bytes are written in pair)
                  ____________ _____ _____ _____ ________
states_fifowriter      0      |  1  |  0  |  1  |  0      (change when r_data_valid is asserted and at negedge of clock_fifowriter)

fifo write happen          o     o     o     o            (posedge of clock_fifowriter while r_data_valid == 1)
*/

reg        r_data_valid;   // changed in negedge of clock_fifowriter  (controlled by SDRAM read submodule)
reg [15:0] r_data;         // changed in negedge of clock_fifowriter  (controlled by SDRAM read submodule)
reg        r_write_single; // changed in negedge of clock_fifowriter  (controlled by SDRAM read submodule)
reg        states_fifowriter, states_fifowriter_next;  // changed in negedge of clock_fifowriter
reg [7:0]  fifo_data;
reg        wen;

assign oFIFO_WCLK = clock_fifowriter;  // push into fifo on the posedge to avoid hazard
assign oFIFO_WEN = wen;
assign oFIFO_WDATA = fifo_data;

always @ (*) begin
	if(r_write_single)
		wen = r_data_valid && (states_fifowriter==1'b0);
	else
		wen = r_data_valid;
end

always @ (*)
case(states_fifowriter)
	// waiting for data_valid. If data_valid is asserted, change state on the negedge 
	// of fifo writer clock and write upper byte into fifo
	1'b0: begin
		states_fifowriter_next = (r_data_valid)? 1'b1: 1'b0;
		fifo_data = r_data[15:8];
	end

	// write lower byte into fifo and return
	1'b1: begin
		states_fifowriter_next = 1'b0;
		fifo_data = r_data[7:0];
	end
endcase

// operate the fifo writing at 2x the sdram clock rate
always @ (posedge clock_fifowriter or posedge iRST) begin
	if(iRST)
		states_fifowriter <= 1'b0;
	else
		states_fifowriter <= states_fifowriter_next;
end





// ============================================
//  VGA data control and SDRAM read submodule
// ============================================

// idle
parameter ST_LISTEN_VGA_REQ                   = 4'd0;
// empty lines
parameter ST_FILL_EMPTY_LINES                 = 4'd1;
// nonempty lines
parameter ST_FILL_HORIZONTAL_BLANK_FRONT_ODD  = 4'd2;
parameter ST_FILL_HORIZONTAL_BLANK_FRONT      = 4'd3;
parameter ST_FILL_DATA_READ                   = 4'd4;
parameter ST_FILL_DATA_READ_STALLED           = 4'd5;
parameter ST_FILL_DATA_READ_ENDING            = 4'd8;  // for the delayed output from SDRAM
parameter ST_FILL_HORIZONTAL_BLANK_BACK_ODD   = 4'd6;
parameter ST_FILL_HORIZONTAL_BLANK_BACK       = 4'd7;

parameter READ_ENDING_WAIT_CYCLES = 3'd5;

reg [3:0]	states, states_next;
reg [10:0]	horizontal_counter, horizontal_counter_next;
reg [2:0]   read_ending_counter, read_ending_counter_next;
reg [8:0]   blank_counter, blank_counter_next;
reg [8:0]   front_blank_count, back_blank_count;
reg         write_single;
reg [9:0]   current_line_id, current_line_id_next;

assign oRD_EN = (states == ST_FILL_DATA_READ) || (states == ST_FILL_DATA_READ_STALLED);
assign oRD_ADDR[24:19] = iFRAME_ID;
assign oRD_ADDR[18:9] = current_line_id;
assign oRD_ADDR[8:0] = horizontal_counter[9:1];


always @ (*) begin
	if(iOFFSET_H_SIGN == 1'b1) begin // negative offset
		front_blank_count = 9'd128 - iOFFSET_H;
		back_blank_count = 9'd128 + iOFFSET_H;
	end
	else begin  // positive offset
		front_blank_count = 9'd128 + iOFFSET_H;
		back_blank_count = 9'd128 - iOFFSET_H;
	end
end


// current_line_id_next
always @ (*)
case(states)
	ST_LISTEN_VGA_REQ: begin
		if(iOFFSET_V_SIGN == 1'b1)  // negative y offset
			current_line_id_next = iVGA_LINE_TO_LOAD[9:0] + iOFFSET_V;
		else  // positive y offset
			current_line_id_next = iVGA_LINE_TO_LOAD[9:0] - iOFFSET_V;
	end
	default:
		current_line_id_next = current_line_id;
endcase


// horizontal_counter_next
always @ (*)
case(states)
	ST_LISTEN_VGA_REQ:          horizontal_counter_next = 11'd0;
	ST_FILL_EMPTY_LINES:        horizontal_counter_next = horizontal_counter + 2'b10;
	//ST_FILL_HORIZONTAL_BLANK_FRONT_ODD
	//ST_FILL_HORIZONTAL_BLANK_FRONT
	ST_FILL_DATA_READ:          horizontal_counter_next = (states_next==ST_FILL_DATA_READ)? horizontal_counter+2'b10 : horizontal_counter;
	ST_FILL_DATA_READ_STALLED:  horizontal_counter_next = (states_next==ST_FILL_DATA_READ)? horizontal_counter+2'b10 : horizontal_counter;
	//ST_FILL_DATA_READ_ENDING
	//ST_FILL_HORIZONTAL_BLANK_BACK_ODD
	//ST_FILL_HORIZONTAL_BLANK_BACK
	default:                    horizontal_counter_next = 11'd0;
endcase


// blank_counter_next
always @ (*)
case(states)
	ST_LISTEN_VGA_REQ:                   blank_counter_next = 9'd0;
	//ST_FILL_EMPTY_LINES
	ST_FILL_HORIZONTAL_BLANK_FRONT_ODD:  blank_counter_next = 9'd1;
	ST_FILL_HORIZONTAL_BLANK_FRONT:      blank_counter_next = blank_counter + 2'b10;
	ST_FILL_DATA_READ:                   blank_counter_next = 9'd0;
	//ST_FILL_DATA_READ_STALLED
	//ST_FILL_DATA_READ_ENDING
	ST_FILL_HORIZONTAL_BLANK_BACK_ODD:   blank_counter_next = 9'd1;
	ST_FILL_HORIZONTAL_BLANK_BACK:       blank_counter_next = blank_counter + 2'b10;
	default:                             blank_counter_next = 9'd0;
endcase


// read_ending_counter_next
always @ (*)
if(states == ST_FILL_DATA_READ_ENDING)
	read_ending_counter_next = read_ending_counter + 1'b1;
else
	read_ending_counter_next = 3'd0;


// states_next
always @ (*)
case(states)
	ST_LISTEN_VGA_REQ: begin
		if(!iVGA_LOAD_TO_FIFO_REQ)
			states_next = ST_LISTEN_VGA_REQ;
		else begin
			if( (iOFFSET_V_SIGN == 1'b1 && iVGA_LINE_TO_LOAD[10:0] >= 11'd1024-iOFFSET_V)||  // negative offset
			    (iOFFSET_V_SIGN == 1'b0 && iVGA_LINE_TO_LOAD[10:0] <  {3'b000,iOFFSET_V}))  // positive offset
			begin  
				states_next = ST_FILL_EMPTY_LINES;
			end
			else
				states_next = (front_blank_count[0]==1'b1)? ST_FILL_HORIZONTAL_BLANK_FRONT_ODD : ST_FILL_HORIZONTAL_BLANK_FRONT;
		end
	end

	ST_FILL_EMPTY_LINES: begin
		states_next = (horizontal_counter_next[10:1] == 10'd640)? ST_LISTEN_VGA_REQ : ST_FILL_EMPTY_LINES;  // 640 due to 1280/2=640
	end

	ST_FILL_HORIZONTAL_BLANK_FRONT_ODD, ST_FILL_HORIZONTAL_BLANK_FRONT: begin
		if(blank_counter_next[8:1]  == front_blank_count[8:1])
			states_next = ST_FILL_DATA_READ;
		else
			states_next = ST_FILL_HORIZONTAL_BLANK_FRONT;
	end

	ST_FILL_DATA_READ, ST_FILL_DATA_READ_STALLED: begin
		if(iWAIT_REQUEST)
            states_next = ST_FILL_DATA_READ_STALLED;
        else begin
            if(horizontal_counter[10:1] == 1024/2-1)  // equivalent to (horizontal_counter_next==11'd1024) under horizontal_counter[0]==0
                states_next = ST_FILL_DATA_READ_ENDING;
            else
                states_next = ST_FILL_DATA_READ;
        end
	end

	ST_FILL_DATA_READ_ENDING: begin
		if(read_ending_counter_next == READ_ENDING_WAIT_CYCLES)
			states_next = (back_blank_count[0]==1'b1)? ST_FILL_HORIZONTAL_BLANK_BACK_ODD : ST_FILL_HORIZONTAL_BLANK_BACK;
		else
			states_next = ST_FILL_DATA_READ_ENDING;
	end

	ST_FILL_HORIZONTAL_BLANK_BACK_ODD, ST_FILL_HORIZONTAL_BLANK_BACK: begin
		if(blank_counter_next[8:1]  == back_blank_count[8:1])
			states_next = ST_LISTEN_VGA_REQ;
		else
			states_next = ST_FILL_HORIZONTAL_BLANK_BACK;
	end

	default: begin
		states_next = ST_LISTEN_VGA_REQ;
	end
endcase


// write_single
always @ (*)
case(states)
	ST_FILL_HORIZONTAL_BLANK_FRONT_ODD, ST_FILL_HORIZONTAL_BLANK_BACK_ODD:
		write_single = 1'b1;
	default:
		write_single = 1'b0;
endcase


// latch the data at the negedge
always @(negedge clock or posedge iRST) begin
	if (iRST) begin
		r_data <= 16'd0;
		r_data_valid <= 1'b0;
		r_write_single <= 1'b0;
	end
	else case(states)
		ST_FILL_EMPTY_LINES,
		ST_FILL_HORIZONTAL_BLANK_FRONT_ODD, ST_FILL_HORIZONTAL_BLANK_FRONT, 
		ST_FILL_HORIZONTAL_BLANK_BACK_ODD, ST_FILL_HORIZONTAL_BLANK_BACK: begin
			r_data <= 16'd0;
			r_data_valid <= 1'b1;
			r_write_single <= write_single;
		end

		ST_FILL_DATA_READ, ST_FILL_DATA_READ_STALLED, ST_FILL_DATA_READ_ENDING: begin
			r_data <= iRD_DATA;
			r_data_valid <= iRD_DATAVALID;
			r_write_single <= write_single;
		end

		default: begin
			r_data <= 16'd0;
			r_data_valid <= 1'b0;
			r_write_single <= 1'b0;
		end
	endcase
end


// main sequential part
always @(posedge clock or posedge iRST) begin
	if(iRST) begin
		states <= ST_LISTEN_VGA_REQ;
		horizontal_counter <= 0;
		blank_counter <= 0;
		read_ending_counter <= 0;
		current_line_id <= 0;
	end
	else begin
		states <= states_next;
		horizontal_counter <= horizontal_counter_next;
		blank_counter <= blank_counter_next;
		read_ending_counter <= read_ending_counter_next;
		current_line_id <= current_line_id_next;
	end
end


endmodule
