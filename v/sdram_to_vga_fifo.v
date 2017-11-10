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
wire clock;
assign clock = iCLK;


// =================================
//  sdram_read FIFO writer
// =================================
/* 
A simple module that stores 16-bit data of SDRAM into a fifo and waits for another module
to convert it into 8-bit data and to store them into VGA fifo.
                   _____       _____       _____       __
clock                   |_____|     |_____|     |_____|
                         ___________ ___________
r_data_valid       _____|           |           |________ (change at negedge of clock if all bytes are written in pair)

fifo write happen             o           o               (posedge of clock while r_data_valid == 1)
*/
reg         r_data_valid;   // changed in negedge of clock  (controlled by SDRAM read submodule)
reg [15:0]  r_data;         // changed in negedge of clock  (controlled by SDRAM read submodule)
reg         r_write_single; // changed in negedge of clock  (controlled by SDRAM read submodule)
wire        sdram_fifo_empty;
wire [16:0] sdram_fifo_data;
reg         sdram_fifo_rdreq;
sdram_read_fifo sdram_read_fifo_0(
	.clock(clock),
	// SDRAM read side
	.data({r_write_single,r_data}),
	.wrreq(r_data_valid),
	// VGA fifo (another fifo) write side
	.rdreq(sdram_fifo_rdreq),
	.empty(sdram_fifo_empty),  // output
	.q(sdram_fifo_data)  // output
);


// =================================
//  VGA FIFO writer
// =================================
/*
                      _____       _____       _____       _____       _____       _____
clock              __|     |_____|     |_____|     |_____|     |_____|     |_____|     |___
                   __                                     _________________________________
sdram_fifo_empty     |___________________________________|
                   ______________ ___________ ___________ ___________ ___________ _________
states_fifowriter  _____idle_____|__w upper__|__w lower__|__w upper__|__w lower__|__idle___
                            ___________             ___________
sdram_fifo_rdreq   ________|           |___________|           |___________________________ (when state is idle or w_lower and sdram fifo not empty)
                                  _______________________ _________________________________
sdram_fifo_data    ______________|_______________________|_________________________________

vga fifo write                         o           o           o           o                (negedges when state is w_upper or w_lower)

*/
parameter VFIFO_ST_IDLE = 3'd0;
parameter VFIFO_ST_WR_U = 3'd1;
parameter VFIFO_ST_WR_L = 3'd2;
reg [2:0]  states_fifowriter, states_fifowriter_next;  // changed in negedge of clock
reg        sdram_fifo_rdreq_next;
reg [7:0]  vga_fifo_data;
reg        vga_fifo_wen;
wire       vga_write_single;

assign oFIFO_WCLK = ~clock;
assign oFIFO_WEN = vga_fifo_wen;
assign oFIFO_WDATA = vga_fifo_data;
assign vga_write_single = sdram_fifo_data[16];

// posedge changes
always @ (*)
case(states_fifowriter)
	VFIFO_ST_IDLE: begin
		states_fifowriter_next = (sdram_fifo_empty)? VFIFO_ST_IDLE : VFIFO_ST_WR_U;
		vga_fifo_data = 8'bxxxxxxxx;
		vga_fifo_wen = 1'b0;
	end
	
	VFIFO_ST_WR_U: begin
		states_fifowriter_next = VFIFO_ST_WR_L;
		vga_fifo_data = sdram_fifo_data[15:8];
		vga_fifo_wen = 1'b1;
	end
	
	VFIFO_ST_WR_L: begin
		states_fifowriter_next = (sdram_fifo_empty)? VFIFO_ST_IDLE : VFIFO_ST_WR_U;
		vga_fifo_data = sdram_fifo_data[7:0];
		vga_fifo_wen = (vga_write_single)? 1'b0 : 1'b1;
	end
	
	default: begin
		states_fifowriter_next = VFIFO_ST_IDLE;
		vga_fifo_data = 8'bxxxxxxxx;
		vga_fifo_wen = 1'b0;
	end
endcase

// negedge changes
always @ (*)
case(states_fifowriter)
	VFIFO_ST_IDLE: begin
		sdram_fifo_rdreq_next = (sdram_fifo_empty)? 1'b0 : 1'b1;
	end
	
	VFIFO_ST_WR_U: begin
		sdram_fifo_rdreq_next = 1'b0;
	end
	
	VFIFO_ST_WR_L: begin
		sdram_fifo_rdreq_next = (sdram_fifo_empty)? 1'b0 : 1'b1;
	end
	
	default: begin
		sdram_fifo_rdreq_next = 1'b0;
	end
endcase

always @ (posedge clock or posedge iRST) begin
	if(iRST)
		states_fifowriter <= 3'd0;
	else
		states_fifowriter <= states_fifowriter_next;
end

always @ (negedge clock or posedge iRST) begin
	if(iRST)
		sdram_fifo_rdreq <= 1'b0;
	else
		sdram_fifo_rdreq <= sdram_fifo_rdreq_next;
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
// negedge required by the usage of r_data, r_data_valid and r_write_single.
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
