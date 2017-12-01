module delay_x00_ms(
	input  iCLOCK50,
	input  iTRIGGER,
	output oDELAY100,  // delayed ~100ms, is held asserted for ~50ms
	output oDELAY200,  // delayed ~200ms, is held asserted for ~50ms
	output oDELAY300,  // delayed ~300ms, is held asserted for ~50ms
	output oDELAY400   // delayed ~400ms, is held asserted for ~50ms
);
// for delayed reset signal
reg  [21:0] local_counter;
reg  [3:0]  halfhundred_ms_counter;
reg  delayed_100, delayed_200, delayed_300, delayed_400;

assign oDELAY100 = delayed_100;
assign oDELAY200 = delayed_200;
assign oDELAY300 = delayed_300;
assign oDELAY400 = delayed_400;

always @ (posedge iCLOCK50) begin
	if (local_counter == 22'd0) begin
		if(iTRIGGER||(halfhundred_ms_counter!=4'd0))
			local_counter <= local_counter + 1'b1;
		else
			local_counter <= 22'd0;
	end
	else if(local_counter >= 22'b10_0110_0010_0101_1010_0000)  // roughly the number of clocks for 0.05 second
		local_counter <= 22'd0;
	else
		local_counter <= local_counter + 1'b1;
	
	if(local_counter >= 22'b10_0110_0010_0101_1010_0000)
		halfhundred_ms_counter <= halfhundred_ms_counter + 1'b1;
	else
		halfhundred_ms_counter <= halfhundred_ms_counter;
end

always @ (*) begin
	if (halfhundred_ms_counter == 4'b0010)  // 4'b0000 is not allowed since an idle counter has this pattern
		delayed_100 = 1'b1;
	else
		delayed_100 = 1'b0;
		
	if (halfhundred_ms_counter == 4'b0100)
		delayed_200 = 1'b1;
	else
		delayed_200 = 1'b0;
	
	if (halfhundred_ms_counter == 4'b0110)
		delayed_300 = 1'b1;
	else
		delayed_300 = 1'b0;
	
	if (halfhundred_ms_counter == 4'b1000)
		delayed_400 = 1'b1;
	else
		delayed_400 = 1'b0;
end

endmodule