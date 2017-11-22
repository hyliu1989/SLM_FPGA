module test_sdram_write(
	input         iCLK,
	input         iRST,

	input         iTRIGGER,
	input         iWAIT_REQUEST,
	output        oWR_EN,
	output [15:0] oWR_DATA,
	output [24:0] oWR_ADDR,
	output        oDONE
);
/*
Writing simple byte patterns into the SDRAM when using module fake_SD_card_FAT32_reader

The 64 frames contain strips at 64 different locations.
This sdram write module should operate faster than the SD card so that the 
data-ready signal (is_data_ready) from SD card read is always captured.
*/

// states
parameter ST_IDLE           = 4'd0;
parameter ST_WAIT_WAITREQ_0 = 4'd4;
parameter ST_WAIT_WAITREQ_1 = 4'd5;
parameter ST_WRITE_WAITDATA = 4'd3;
parameter ST_WRITE_REQ      = 4'd1;
parameter ST_WRITE_STALLED  = 4'd2;
parameter ST_DONE_AND_WAIT  = 4'd15;
// states for ready-signal capturing
parameter RDST_WAITNEW             = 2'b00;
parameter RDST_WAIT_RAM_CAPTURE    = 2'b01;
parameter RDST_WAIT_READER_RETURN  = 2'b10;


// local signals
reg  [3:0]  states, states_next;
reg  [24:0] counter, counter_next, counter_plus_1;
reg  [15:0] data, data_next;
wire [24:0] addr;
wire        is_data_ready;
reg         is_last_data, is_last_data_next;
reg  [1:0]  data_ready_capture_states, data_ready_capture_states_next;

// reader signals
wire        reader_data_ready;
wire        reader_last_data;
wire [15:0] data_from_reader;

// =======================================
//  Main
// =======================================
assign addr = counter;
assign oWR_ADDR = addr;
assign oWR_EN = (states == ST_WRITE_REQ) || (states == ST_WRITE_STALLED);
assign oWR_DATA = data;
assign oDONE = (states == ST_DONE_AND_WAIT) || (states == ST_IDLE);
assign is_data_ready = (data_ready_capture_states == RDST_WAIT_RAM_CAPTURE);

fake_SD_card_FAT32_reader f0(
	.iCLK(iCLK),
	.iRST(iRST),

	.iTRIGGER(iTRIGGER),

	.oDATA(data_from_reader),
	.oDATA_READY(reader_data_ready),
	.oLAST_DATA(reader_last_data)
	);

// data_ready_capture_states_next
always @ (*)
case(data_ready_capture_states)
	// waiting for a new ready signal
	RDST_WAITNEW:
		data_ready_capture_states_next = (reader_data_ready)? RDST_WAIT_RAM_CAPTURE : RDST_WAITNEW;

	// SD card reader ready, RAM module has not captured
	// It is this part that requires the RAM module to write faster than the  the reading of SD card
	// module. Otherwise the fall of reader ready signal (in the next state RDST_WAIT_READER_RETURN)
	// might be missed.
	RDST_WAIT_RAM_CAPTURE:
		data_ready_capture_states_next = (states_next==ST_WRITE_REQ)? RDST_WAIT_READER_RETURN : RDST_WAIT_RAM_CAPTURE;

	// waiting for the reader's ready signal to fall after RAM module has captured the ready signal
	RDST_WAIT_READER_RETURN:
		data_ready_capture_states_next = (!reader_data_ready)? RDST_WAITNEW : RDST_WAIT_READER_RETURN;

	// default
	default:
		data_ready_capture_states_next = RDST_WAITNEW;
endcase


// states_next
always @ (*)
case(states)
	// idle
	ST_IDLE:
		states_next = (iTRIGGER)? ST_WRITE_WAITDATA : ST_IDLE;
		
	ST_WAIT_WAITREQ_0:
		states_next = (iWAIT_REQUEST)? ST_WAIT_WAITREQ_0 : ST_WAIT_WAITREQ_1;
	
	ST_WAIT_WAITREQ_1:
		states_next = ST_WRITE_WAITDATA;

	// wait for data to be ready
	ST_WRITE_WAITDATA:
		states_next = (is_data_ready)? ST_WRITE_REQ : ST_WRITE_WAITDATA;
	
	// ST_WRITE_REQ: send write request
	// ST_WRITE_STALLED: waiting for the sdram controller to proceed
	ST_WRITE_REQ, ST_WRITE_STALLED: begin
		if(iWAIT_REQUEST)
			states_next = ST_WRITE_STALLED;
		else begin
			if(is_last_data) 
				states_next = ST_DONE_AND_WAIT;
			else
				states_next = ST_WRITE_WAITDATA;
		end
	end
	
	//  wait until the end of trigger signal
	ST_DONE_AND_WAIT:
		states_next = (!iTRIGGER)? ST_IDLE : ST_DONE_AND_WAIT;
	
	// default
	default:
		states_next = ST_IDLE;
endcase

// is_last_data_next
always @ (*) begin
	case(states)
		ST_WRITE_WAITDATA:          is_last_data_next = reader_last_data;
		ST_IDLE,ST_DONE_AND_WAIT:   is_last_data_next = 1'b0;  // False
		default:                    is_last_data_next = is_last_data;
	endcase
end

// counter_next
always @ (*) begin
	counter_plus_1 = counter + 1'b1;  // constantly +1 signal

	case(states)
		ST_WRITE_WAITDATA:  counter_next = (states_next==ST_WRITE_REQ)? counter_plus_1 : counter;
		ST_WRITE_REQ:       counter_next = counter;
		ST_WRITE_STALLED:   counter_next = counter;
		default:            counter_next = 25'h1ff_ffff;
	endcase
end

// data_next
always @ (*) begin
	case(states)
		ST_WRITE_WAITDATA:  data_next = (is_data_ready)? data_from_reader : data;
		default:            data_next = data;
	endcase
end


// sequential part
always @ (posedge iRST or posedge iCLK) begin
	if(iRST) begin
		states <= ST_IDLE;
		counter <= 9'd0;
		data <= 0;
		data_ready_capture_states <= RDST_WAITNEW;
		is_last_data <= 1'b0;
	end
	else begin
		states <= states_next;
		counter <= counter_next;
		data <= data_next;
		data_ready_capture_states <= data_ready_capture_states_next;
		is_last_data <= is_last_data_next;
	end
end

endmodule



module fake_SD_card_FAT32_reader(
	input         iCLK,
	input         iRST,

	input         iTRIGGER,

	output [15:0] oDATA,
	output        oDATA_READY,
	output        oLAST_DATA
	);

// Simulate a slow SD card module by slowing the clock
reg [2:0] clock_counter;
wire CLOCK;
always @ (posedge iCLK) begin
	clock_counter <= clock_counter + 1'b1;
end
assign CLOCK = clock_counter[2];



// states
parameter ST_IDLE = 4'd0;
parameter ST_PREPARE_DATA = 4'd1;
parameter ST_SEND_REQ = 4'd2;
parameter ST_PREP_LAST = 4'd3;
parameter ST_REQ_LAST = 4'd4;
parameter ST_DONE_AND_WAIT = 4'd5;

parameter STRIP_WIDTH = 12;  // in unit of pixels

reg [3:0]         states, states_next;
reg [24:0]        counter, counter_next;
reg [15:0]        data, data_next;
wire [5:0]        frame_id;
wire [9:0]        line_id;
wire [8:0]        twobytes_id;
reg [15:0]        rDATA;
reg               rDATA_READY;
reg               rLAST_DATA;

assign frame_id = counter[24:19];
assign line_id = counter[18:9];
assign twobytes_id = counter[8:0];

always @ (posedge CLOCK) begin
	// retarding the output signals
	rDATA <= data;
	rDATA_READY <= (states == ST_SEND_REQ) || (states == ST_REQ_LAST);
	rLAST_DATA <= (states == ST_REQ_LAST);
end
assign oDATA = rDATA;
assign oDATA_READY = rDATA_READY;
assign oLAST_DATA = rLAST_DATA;

// states_next
always @ (*)
case(states)
	ST_IDLE:          states_next = (iTRIGGER)? ST_PREPARE_DATA : ST_IDLE;
	ST_PREPARE_DATA:  states_next = ST_SEND_REQ;
	ST_SEND_REQ:      states_next = (counter == 25'h1ff_fffe)? ST_PREP_LAST : ST_PREPARE_DATA;
	ST_PREP_LAST:     states_next = ST_REQ_LAST;
	ST_REQ_LAST:      states_next = ST_DONE_AND_WAIT;
	ST_DONE_AND_WAIT: states_next = (!iTRIGGER)? ST_IDLE : ST_DONE_AND_WAIT;
	default:          states_next = ST_IDLE;
endcase

// counter_next
always @ (*)
case(states)
	ST_IDLE:          counter_next = 25'd0;
	ST_PREPARE_DATA:  counter_next = counter;
	ST_SEND_REQ:      counter_next = counter + 1'b1;
	ST_PREP_LAST:     counter_next = counter;
	ST_REQ_LAST:      counter_next = counter;
	default:          counter_next = 25'd0;
endcase

// data_next
always @ (*)
case(states)
	ST_IDLE:
		data_next = 16'd0;
	ST_PREPARE_DATA, ST_PREP_LAST: begin
		if(line_id == 10'd0)
			data_next = 16'hffff;
		else if((twobytes_id >= frame_id*8) && (twobytes_id < frame_id*8 + STRIP_WIDTH/2))
			data_next = {2'b11, line_id[9:7],3'b111, 2'b11,line_id[9:7],3'b111};
		else
			data_next = 16'd0;
	end
	default:
		data_next = data;
endcase

// sequential part
always @ (posedge iRST or posedge CLOCK) begin
	if(iRST) begin
		states <= ST_IDLE;
		counter <= 25'd0;
		data <= 16'd0;
	end
	else begin
		states <= states_next;
		counter <= counter_next;
		data <= data_next;
	end
end	
	
endmodule
