module test_sdram_read(
	input         iCLK,
	input         iRST,
	
	input         iTEST_WRITE_DONE,
	
	input         iWAIT_REQUEST,
	output        oRD_EN,
	output [24:0] oRD_ADDR,
	input  [15:0] iRD_DATA,
	input         iRD_DATAVALID,
	
	input  [9:0]  iSW,
	output [7:0]  oDATA
);
/*
Reading the data in sdram
*/

// states
parameter ST_IDLE = 4'd0;
parameter ST_READ_REQ = 4'd1;
parameter ST_READ_STALLED = 4'd2;
parameter ST_READ_DONE = 4'd15;


reg  [3:0]  states, states_next;
reg  [19:0] counter_idle, counter_idle_next;
reg  [15:0] data_captured;

assign oRD_ADDR[24:9] = 16'd0;
assign oRD_ADDR[8:0] = iSW[9:1];
assign oRD_EN = (states == ST_READ_REQ) || (states == ST_READ_STALLED);
assign oDATA = (iSW[0] == 0)? data_captured[15:8] : data_captured[7:0];


// states_next
always @ (*) case(states)
	ST_IDLE:  // falling edge of write_done triggers the reading
		states_next = (counter_idle==0 && iTEST_WRITE_DONE)? ST_READ_REQ:ST_IDLE;
	
	ST_READ_REQ: begin  //  send write request
		if(iWAIT_REQUEST)
			states_next = ST_READ_STALLED;
		else begin
			states_next = ST_READ_DONE;
		end
	end
	
	ST_READ_STALLED: begin  // holding the counter because the wait request
		if(iWAIT_REQUEST)
			states_next = ST_READ_STALLED;
		else begin
			states_next = ST_READ_DONE;
		end
	end
	
	default:
		states_next = ST_IDLE;
endcase

always @ (*) begin
	counter_idle_next = counter_idle + 1'b1;
end


// sequential part
always @ (posedge iRST or posedge iCLK) begin
	if(iRST) begin
		states <= ST_IDLE;
		counter_idle <= 1;
	end
	else begin
		states <= states_next;
		counter_idle <= counter_idle_next;
	end
end

// output of the result of reading operation
always @ (posedge iRST or negedge iCLK) begin
	if(iRST) begin
		data_captured <= 16'd0;
	end
	else begin
		if(iRD_DATAVALID) begin
			data_captured <= iRD_DATA;
		end
	end
end

endmodule