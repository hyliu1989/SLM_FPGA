module write_to_sdram(
	input         iCLK,
	input         iRST,

	input         iTRIGGER,
    
    // SDRAM Avalon signals
	input         iWAIT_REQUEST,
	output        oWR_REQ,
	output [15:0] oWR_DATA,
	output [24:0] oWR_ADDR,
	output        oDONE,
    
    // signals from the FIFO that contains data_out
    output        oFIFO_RD_CLK,
    output        oFIFO_RD_REQ,
    input  [7:0]  iFIFO_RD_DATA,
    input         iFIFO_RD_EMPTY,
    input  [6:0]  iNUM_IMAGES,
    input  [5:0]  iID_OF_STARTING_IMAGE
);
/*
Writing simple byte patterns into the SDRAM when using module fake_SD_card_FAT32_reader

The 64 frames contain strips at 64 different locations.
This sdram write module should operate faster than the SD card so that the 
data_out-ready signal (is_data_ready) from SD card read is always captured.
*/

// states
parameter ST_IDLE           = 4'd0;
parameter ST_WAIT_WAITREQ_0 = 4'd1;
parameter ST_WAIT_WAITREQ_1 = 4'd2;
parameter ST_READ0_WAITDATA = 4'd3;
parameter ST_READ0_REQUEST  = 4'd4;
parameter ST_READ1_WAITDATA = 4'd5;
parameter ST_READ1_REQUEST  = 4'd6;
parameter ST_READ_COMBINE   = 4'd7;
parameter ST_WRITE_REQ      = 4'd8;
parameter ST_DONE_AND_WAIT  = 4'd15;

parameter state_following_ST_READ0_REQUEST = ST_READ1_WAITDATA;
parameter state_following_ST_READ1_REQUEST = ST_READ_COMBINE;

// local signals
reg  [3:0]  states, states_next;
reg  [24:0] counter, counter_next, r0_counter_p1, r1_counter_p1;
reg  [15:0] data_out, data_out_next;
reg  [24:0] addr, addr_next;
wire        is_data_ready;
wire [7:0]  data_in;

reg  [6:0]  num_images, num_images_next;
reg  [5:0]  starting_image_id, starting_image_id_next;
wire        is_last_data;


// =======================================
//  Main
// =======================================
assign oWR_ADDR = addr;
assign oWR_REQ = (states == ST_WRITE_REQ);
assign oWR_DATA = data_out;
assign oDONE = (states == ST_DONE_AND_WAIT) || (states == ST_IDLE);

assign is_data_ready = !iFIFO_RD_EMPTY;
assign data_in = iFIFO_RD_DATA;
assign oFIFO_RD_REQ = (states == ST_READ0_REQUEST) || (states == ST_READ1_REQUEST);
assign oFIFO_RD_CLK = iCLK;
assign is_last_data = (r1_counter_p1 == {starting_image_id+num_images[5:0], 19'd0});


// states_next
always @ (*)
case(states)
	// idle
	ST_IDLE:            states_next = (iTRIGGER)? ST_WAIT_WAITREQ_0 : ST_IDLE;
	ST_WAIT_WAITREQ_0:  states_next = (iWAIT_REQUEST)? ST_WAIT_WAITREQ_0 : ST_WAIT_WAITREQ_1;
	ST_WAIT_WAITREQ_1:  states_next = ST_READ0_WAITDATA;
	ST_READ0_WAITDATA:  states_next = (is_data_ready)? ST_READ0_REQUEST : ST_READ0_WAITDATA;
    ST_READ0_REQUEST:   states_next = ST_READ1_WAITDATA;
    ST_READ1_WAITDATA:  states_next = (is_data_ready)? ST_READ1_REQUEST : ST_READ1_WAITDATA;
    ST_READ1_REQUEST:   states_next = ST_READ_COMBINE;
    ST_READ_COMBINE:    states_next = ST_WRITE_REQ;
	
	// ST_WRITE_REQ: send write request
	ST_WRITE_REQ: begin
		if(iWAIT_REQUEST)
			states_next = ST_WRITE_REQ;
		else begin
			if(is_last_data)
				states_next = ST_DONE_AND_WAIT;
			else
				states_next = ST_READ0_WAITDATA;
		end
	end

	ST_DONE_AND_WAIT:   states_next = (!iTRIGGER)? ST_IDLE : ST_DONE_AND_WAIT;  //  wait until the end of trigger signal
	
	default:            states_next = ST_IDLE;
endcase


// counter_next, addr_next, r0_counter_p1, r1_counter_p1
always @ (*) begin
	case(states)
        ST_IDLE:            counter_next = (iTRIGGER)? {iID_OF_STARTING_IMAGE, 19'd0} : counter;
		ST_WRITE_REQ:       counter_next = (states_next==ST_READ0_WAITDATA)? r1_counter_p1 : counter;  // r1_counter_p1 is a retarded (counter + 1)
		default:            counter_next = counter;
	endcase
    
    case(states)
        ST_READ_COMBINE:    addr_next = counter;
        default:            addr_next = addr;
    endcase
    
    if(states==ST_IDLE && iTRIGGER) begin
        starting_image_id_next = iID_OF_STARTING_IMAGE;
        num_images_next = iNUM_IMAGES;
    end
    else begin
        starting_image_id_next = starting_image_id;
        num_images_next = num_images;
    end
end
always @ (posedge iCLK) begin
    r0_counter_p1 <= counter + 1'b1;
    r1_counter_p1 <= r0_counter_p1;
end


// data_out_next
always @ (*) begin
	case(states)
        // the state following ST_READ0_REQUEST
		state_following_ST_READ0_REQUEST:  data_out_next = {data_in,        data_out[7:0]};
        state_following_ST_READ1_REQUEST:  data_out_next = {data_out[15:8], data_in      };
		default:                           data_out_next = data_out;
	endcase
end




// sequential part
always @ (posedge iRST or posedge iCLK) begin
	if(iRST) begin
		states <= ST_IDLE;
		counter <= 25'd0;
        addr <= 25'd0;
		data_out <= 0;
        starting_image_id <= 0;
        num_images <= 0;
	end
	else begin
		states <= states_next;
		counter <= counter_next;
        addr <= addr_next;
		data_out <= data_out_next;
        starting_image_id <= starting_image_id_next;
        num_images <= num_images_next;
	end
end

endmodule