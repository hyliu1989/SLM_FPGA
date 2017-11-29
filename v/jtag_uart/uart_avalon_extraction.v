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
This module extract the data from Avalon interface for processing.

                            ____      ____      ____      ____      ____
clock                 _____|    |____|    |____|    |____|    |____|    |____
                            _____________________________________
req                   _____|
                            _________ 
read_wait             _____|         |______________________________
                      _______________ _________ _________ _________
data_from_uart        _______________|____1____|____2____|____3____|   Avalon latency is 0 cycle after a deasserted read_wait.
                                      ______________________________
data_from_uart_ready  _______________|
                      _________________________ _________ _________
data_to_parse         _________________________|____1____|____2____|
                                                _________ _________
data_to_parse_valid   _________________________|____X____|____X____|   depends on the valid bit of data_from_uart

*/
parameter ST_UART_IDLE =         2'b00;
parameter ST_UART_READ =         2'b01;
reg [1:0]  states_uart;
wire       data_from_uart_ready;
wire [7:0] data_from_uart;
wire       data_from_uart15;
reg [7:0]  data_to_parse;
reg        data_to_parse_valid;

assign data_from_uart_ready = (states_uart == ST_UART_READ) && (!iJTAG_SLAVE_WAIT);
assign data_from_uart = iJTAG_SLAVE_RDDATA[7:0];
assign data_from_uart15 = iJTAG_SLAVE_RDDATA[15];

always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states_uart <= ST_UART_IDLE;
        data_to_parse_valid <= 1'b0;
        data_to_parse <= 8'd0;
    end
    else begin
        states_uart <= ST_UART_READ;
        data_to_parse_valid <= (data_from_uart_ready && data_from_uart15);
        data_to_parse <= data_from_uart;
    end
end

assign oJTAG_SLAVE_ADDR = 1'b0;  // always look at the DATA fifo
assign oJTAG_SLAVE_RDREQ = (states_uart == ST_UART_READ);
assign oJTAG_SLAVE_WRREQ = 1'b0;
assign oJTAG_SLAVE_WRDATA = 32'd0;

assign oDATA_TO_PARSE = data_to_parse;
assign oDATA_TO_PARSE_VALID = data_to_parse_valid;

endmodule