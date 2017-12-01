module millisec_counter(
    input       iCLOCK50,
    input       iRST,
    input       iTRIGGER,
    input [7:0] iMILLISEC_TO_COUNT,
    output      oPULSE  // pulse of length specified by iMILLISEC_TO_COUNT
);
// Counting starts at the falling edge of iTRIGGER

reg  [15:0] local_counter;
reg  [7:0]  m_millisec_to_count;
reg  [7:0]  current_millisec_count;
reg         counting;

always @ (posedge iCLOCK50 or posedge iRST) begin
    if(iRST) begin
        local_counter <= 0;
        current_millisec_count <= 0;
    end
    else begin
    	if(iTRIGGER)
    		m_millisec_to_count <= iMILLISEC_TO_COUNT;
    	else
    		m_millisec_to_count <= m_millisec_to_count;

        if(iTRIGGER)
            local_counter <= 0;
        else if(local_counter >= 16'b1100_0011_0100_1111)  // 16'b1100_0011_0101_0000 = 50000 and 50000 clock cycle == 1 ms if clock freq is 50MHz
            local_counter <= 0;
        else
            local_counter <= local_counter + 1'b1;

        if(iTRIGGER)
            current_millisec_count <= 8'd0;
        else begin
        	if(counting) begin
	        	if(local_counter == 16'b1100_0011_0100_1111)
	            	current_millisec_count <= current_millisec_count + 1'b1;
	            else
	            	current_millisec_count <= current_millisec_count;
	        end
	        else
	        	current_millisec_count <= 0;
	    end

	    if(iTRIGGER)
	    	counting <= 1'b1;
	   	else if(current_millisec_count == m_millisec_to_count)
	   		counting <= 1'b0;
	   	else
	   		counting <= counting;
    end
end

assign oPULSE = counting;


endmodule
