module jtag_uart_decode(
    input         iCLK,
	input         iRST,
    
    // jtag uart signals
    output        oJTAG_SLAVE_ADDR,
    output        oJTAG_SLAVE_RDREQ,
    input [31:0]  iJTAG_SLAVE_RDDATA,
    output        oJTAG_SLAVE_WRREQ,
    output [31:0] oJTAG_SLAVE_WRDATA,
    input         iJTAG_SLAVE_WAIT,
    
    // decoded signals
    input         iDECODEDIMAGE_RDFIFO_CLK,
    input         iDECODEDIMAGE_RDFIFO_REQ,
    output [7:0]  oDECODEDIMAGE_RDFIFO_DATA,
    output        oDECODEDIMAGE_RDFIFO_EMPTY
    
    ,output [7:0]  oTest
);


wire        fifo_wrreq;
wire [7:0]  fifo_wrdata;
wire [7:0]  data_to_parse;
wire        data_to_parse_valid;
reg  [7:0]  test_signals;

always @ (posedge iCLK)
    if(data_to_parse_valid)
        test_signals <= data_to_parse;
assign oTest = test_signals;

uart_avalon_extraction uart_avalon_extraction_0(
    .iCLK(iCLK),
    .iRST(iRST),
    .oJTAG_SLAVE_ADDR(oJTAG_SLAVE_ADDR),
    .oJTAG_SLAVE_RDREQ(oJTAG_SLAVE_RDREQ),
    .iJTAG_SLAVE_RDDATA(iJTAG_SLAVE_RDDATA),
    .oJTAG_SLAVE_WRREQ(oJTAG_SLAVE_WRREQ),
    .oJTAG_SLAVE_WRDATA(oJTAG_SLAVE_WRDATA),
    .iJTAG_SLAVE_WAIT(iJTAG_SLAVE_WAIT),
    .oDATA_TO_PARSE(data_to_parse),
    .oDATA_TO_PARSE_VALID(data_to_parse_valid)
);

// The following fifo stores the image data to be store into SDRAM.
// The special character 0xFE is the value of a pixel and does not
// have the meaning of the escaping character.
fifo_vga fifo_for_pixels(
    .aclr(iRST),
    .wrclk(iCLK),
    .wrreq(fifo_wrreq),
    .data(fifo_wrdata),
    .wrfull(),

    .rdclk(iDECODEDIMAGE_RDFIFO_CLK),
    .rdreq(iDECODEDIMAGE_RDFIFO_REQ),
    .q(oDECODEDIMAGE_RDFIFO_DATA),
    .rdempty(oDECODEDIMAGE_RDFIFO_EMPTY)
);


reg [6:0]   states, states_next;
reg         received_0xFE, received_0xFE_next;
parameter ST_IDLE =             7'h00;
parameter ST_ERROR =            7'h7F;

/*
always @ (*) begin
    if(data_to_parse_valid) begin
        if(received_0xFE)
            received_0xFE_next = 1'b0;
        else begin
            received_0xFE_next = (data_to_parse == 8'hFE);
        end

        if(!received_0xFE) begin
            if(data_to_parse == 8'hFE)  // encountering 0xFE, hold the position
                states_next = states;
            else case(states)  // normal behavior
                // TODO
            endcase
        end
        else begin  // 0xFE interruption behavior
            case(data_to_parse)
                // TODO
                8'h00:   states_next = ST_IDLE;
                8'hFE: begin
                    case(states)
                        // TODO
                    endcase
                end
                default: states_next = ST_ERROR;
            endcase
        end
    end
    else begin
        states_next = states;
        received_0xFE_next = received_0xFE;
    end
end
*/

// main sequential part
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states <= ST_IDLE;
        received_0xFE <= 1'b0;
    end
    else begin
        states <= states_next;
        received_0xFE <= received_0xFE_next;
    end
end




endmodule






module uart_avalon_extraction(
    input iCLK,
    input iRST,

    // jtag uart signals
    output        oJTAG_SLAVE_ADDR,
    output        oJTAG_SLAVE_RDREQ,
    input [31:0]  iJTAG_SLAVE_RDDATA,
    output        oJTAG_SLAVE_WRREQ,
    output [31:0] oJTAG_SLAVE_WRDATA,
    input         iJTAG_SLAVE_WAIT,

    // data to parse
    output [7:0]  oDATA_TO_PARSE,
    output        oDATA_TO_PARSE_VALID

);
/*
                            ____      ____      ____      ____      ____
clock                 _____|    |____|    |____|    |____|    |____|    |____
                            _____________________________________
req                   _____|
                            _________ 
read_wait             _____|         |______________________________
                      _________________________ _________ _________ _________
data_from_uart        _________________________|____1____|____2____|____3____|   assuming Avalon latency is 1 cycle of deasserted read_wait.
                                                ______________________________
data_from_uart_ready  _________________________|
                      ___________________________________ _________ _________
data_to_parse         ___________________________________|____1____|____2____|
                                                          _________ _________
data_to_parse_valid   ___________________________________|____X____|____X____|   depends on the valid bit of data_from_uart

*/
parameter ST_UART_IDLE =         2'b00;
parameter ST_UART_READ =         2'b01;
reg [1:0]  states_uart;
reg        data_from_uart_ready, data_from_uart_ready_next;
reg [7:0]  data_to_parse;
reg        data_to_parse_valid;

always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states_uart <= ST_UART_IDLE;
        data_from_uart_ready <= 1'b0;
        data_to_parse_valid <= 1'b0;
        data_to_parse <= 8'd0;
    end
    else begin
        states_uart <= ST_UART_READ;
        data_from_uart_ready <= (states_uart == ST_UART_READ) && (!iJTAG_SLAVE_WAIT);
        data_to_parse_valid <= (data_from_uart_ready && iJTAG_SLAVE_RDDATA[15]);
        data_to_parse <= iJTAG_SLAVE_RDDATA[7:0];
    end
end

assign oJTAG_SLAVE_ADDR = 1'b0;  // always look at the DATA fifo
assign oJTAG_SLAVE_RDREQ = (states_uart == ST_UART_READ);
assign oJTAG_SLAVE_WRREQ = 1'b0;
assign oJTAG_SLAVE_WRDATA = 32'd0;

assign oDATA_TO_PARSE = data_to_parse;
assign oDATA_TO_PARSE_VALID = data_to_parse_valid;

endmodule
