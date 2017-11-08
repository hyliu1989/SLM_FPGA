module test_sdram_write(
	input         iCLK,
	input         iRST,
	
	input         iWAIT_REQUEST,
	output        oWR_EN,
	output [15:0] oWR_DATA,
	output [24:0] oWR_ADDR,
	output        oDONE
);
/*
Writing simple byte patterns into the SDRAM
The content of each byte is its address mode 256, with the MSB flipped. 
(byte address = word address multiply by 2 and +0 for upper byte and +1 for lower byte)
*/

// states
parameter ST_IDLE = 4'd0;
parameter ST_WRITE_REQ = 4'd1;
parameter ST_WRITE_STALLED = 4'd2;
parameter ST_DONE_AND_WAIT = 4'd15;

parameter n_words_to_write = 9'd400;  // a word contains two bytes here
//assert(n_words_to_write < 512);

reg  [3:0]  states, states_next;
reg  [8:0]  counter, counter_next;
reg  [15:0] data, data_next;
wire [24:0] addr;

assign addr[24:9] = 16'd0;
assign addr[8:0] = counter;
assign oWR_ADDR = addr;
assign oWR_EN = (states == ST_WRITE_REQ) || (states == ST_WRITE_STALLED);
assign oWR_DATA = data;
assign oDONE = (states == ST_DONE_AND_WAIT) || (states == ST_IDLE);

// states_next
always @ (*)
case(states)
	ST_IDLE: begin  //  idle
		if(iRST)
			states_next = ST_WRITE_REQ;
		else
			states_next = ST_IDLE;
	end
	
	ST_WRITE_REQ: begin  //  send write request
		if(iWAIT_REQUEST)
			states_next = ST_WRITE_STALLED;
		else begin
			if(counter == n_words_to_write-1) 
				states_next = ST_DONE_AND_WAIT;
			else
				states_next = ST_WRITE_REQ;
		end
	end
	
	ST_WRITE_STALLED: begin  // holding the counter because the wait request
		if(iWAIT_REQUEST)
			states_next = ST_WRITE_STALLED;
		else begin
			if(counter == n_words_to_write-1) 
				states_next = ST_DONE_AND_WAIT;
			else
				states_next = ST_WRITE_REQ;
		end
	end
	
	ST_DONE_AND_WAIT: begin  //  wait until the end of reset signal
		if(iRST)
			states_next = ST_DONE_AND_WAIT;
		else
			states_next = ST_IDLE;
		end
	
	default:
		states_next = ST_IDLE;
endcase

// counter_next
always @ (*)
case(states)
	default:  counter_next = 9'd0;
	
	ST_WRITE_REQ:  begin
		if(states_next == ST_WRITE_REQ)
			counter_next = counter + 1'b1;
		else
			counter_next = counter;
	end
	ST_WRITE_STALLED:  begin
		if(states_next == ST_WRITE_REQ)
			counter_next = counter + 1'b1;
		else
			counter_next = counter;
	end
endcase

// data_next
always @ (*) begin
	data_next[15:8] = {counter_next[6:3],~counter_next[2], counter_next[1:0], 1'b0};
	data_next[7:0]  = {counter_next[6:3],~counter_next[2], counter_next[1:0], 1'b1};
end


// sequential part
always @ (posedge iCLK) begin
	states <= states_next;
	counter <= counter_next;
	data <= data_next;
end

endmodule
