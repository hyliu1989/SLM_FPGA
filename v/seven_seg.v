module seven_seg(input [3:0] number,  output [6:0] display);

reg [6:0] m_output;

always @ (*) begin
case(number)
    4'd0:  m_output = 7'b0111111;
    4'd1:  m_output = 7'b0000110;
    4'd2:  m_output = 7'b1011011;
    4'd3:  m_output = 7'b1001111;
    4'd4:  m_output = 7'b1100110;
    4'd5:  m_output = 7'b1101101;
    4'd6:  m_output = 7'b1111101;
    4'd7:  m_output = 7'b0000111;
    4'd8:  m_output = 7'b1111111;
    4'd9:  m_output = 7'b1101111;
    4'd10: m_output = 7'b1110111;
    4'd11: m_output = 7'b1111100;
    4'd12: m_output = 7'b1011000;
    4'd13: m_output = 7'b1011110;
    4'd14: m_output = 7'b1111001;
    4'd15: m_output = 7'b1110001;
endcase
end

assign display = ~m_output;

endmodule