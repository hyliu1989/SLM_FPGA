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
    output        oDECODEDIMAGE_RDFIFO_EMPTY,
    output [6:0]  oNUM_IMAGES,
    output        oTRIGGER_WRITE_SDRAM,
    
    output        oERROR
);

// ====================================================
//   Communicating with JTAG-UART core
// ====================================================
wire [7:0]  data_or_cmd;
wire        data_or_cmd_valid;
uart_avalon_extraction uart_avalon_extraction_0(
    .iCLK(iCLK),
    .iRST(iRST),
    .oJTAG_SLAVE_ADDR(oJTAG_SLAVE_ADDR),
    .oJTAG_SLAVE_RDREQ(oJTAG_SLAVE_RDREQ),
    .iJTAG_SLAVE_RDDATA(iJTAG_SLAVE_RDDATA),
    .oJTAG_SLAVE_WRREQ(oJTAG_SLAVE_WRREQ),
    .oJTAG_SLAVE_WRDATA(oJTAG_SLAVE_WRDATA),
    .iJTAG_SLAVE_WAIT(iJTAG_SLAVE_WAIT),
    .oDATA_TO_PARSE(data_or_cmd),
    .oDATA_TO_PARSE_VALID(data_or_cmd_valid)
);


reg [6:0]   states, states_next;
reg         new_instr_ack, new_data_ack;
parameter ST_IDLE                              = 7'h00;
parameter INSTRUCTION_ACK                      = 7'h01;  // not a state but just an acknowledge instruction
parameter ST_ERROR                             = 7'h0F;
parameter ST_UPDATE_RAM_get_num_of_frames      = 7'h10;


// =============================================================
//   Data parsing to extract state instruction and actual data
// =============================================================
reg         received_0xFE, received_0xFE_next;
reg [6:0]   state_instuction, state_instuction_next;  // an instruction to states, `state_instuction` will be check as `states` progresses.
reg         is_there_new_instruct, is_there_new_instruct_next;
reg [7:0]   data, data_next;
reg         is_there_new_data, is_there_new_data_next;

always @ (*) begin
    // received_0xFE_next
    if(data_or_cmd_valid) begin
        if(received_0xFE)
            received_0xFE_next = 1'b0;
        else
            received_0xFE_next = (data_or_cmd == 8'hFE);
    end
    else
        received_0xFE_next = received_0xFE;
    
    
    // state_instuction_next, is_there_new_instruct_next, data_next, is_there_new_data_next
    if(data_or_cmd_valid) begin
        // 0xFE interruption behavior
        if(received_0xFE) begin
            // For instruction to the states
            case(data_or_cmd)
                8'h00: begin
                    state_instuction_next = ST_IDLE;
                    is_there_new_instruct_next = 1'b1;
                end
                8'h01: begin
                    state_instuction_next = INSTRUCTION_ACK;
                    is_there_new_instruct_next = 1'b1;
                end
                8'hFE: begin
                    state_instuction_next = 7'bxxx_xxxx;
                    is_there_new_instruct_next = 1'b0;  // no new state to move to
                end
                default: begin
                    state_instuction_next = ST_ERROR;
                    is_there_new_instruct_next = 1'b1;
                end
            endcase
            
            // For data, the only data after a 0xFE escaping
            data_next              = (data_or_cmd == 8'hFE)? 8'hFE : 8'bxxxx_xxxx;
            is_there_new_data_next = (data_or_cmd == 8'hFE)? 1'b1  : 1'b0;
        end

        // normal behavior
        else begin
            data_next = data_or_cmd;
            // When Idle or Idle with Error, take instructions
            if(states == ST_IDLE || states == ST_ERROR) begin
                casez(data_or_cmd)
                    // Kick off transferring data to SDRAM
                    8'b10??_????: begin
                        state_instuction_next = ST_UPDATE_RAM_get_num_of_frames;
                        is_there_new_instruct_next = 1'b1;
                        is_there_new_data_next = 1'b1;  // the data in this case is the number of frames to transfer minus 1.
                    end

                    // Kick of other processes: TODO

                    // Bypass the escaping
                    8'hFE: begin
                        state_instuction_next = 7'bxxx_xxxx;
                        is_there_new_instruct_next = 1'b0;
                        is_there_new_data_next = 1'b0;
                    end

                    // otherwise, go to the error state
                    default: begin
                        state_instuction_next = ST_ERROR;
                        is_there_new_instruct_next = 1'b1;
                        is_there_new_data_next = 1'b0;
                    end
                endcase
            end
            // When machining is in some processing, no instruction but obtain the data.
            else begin
                state_instuction_next = 7'bxxx_xxxx;
                is_there_new_instruct_next = 1'b0;
                is_there_new_data_next = 1'b1;
            end
        end
    end
    // when data_or_cmd is not valid, mostly hold the positions of the 4 registers
    else begin
        state_instuction_next = state_instuction;
        is_there_new_instruct_next = (new_instr_ack)? 1'b0 : is_there_new_instruct;
        
        data_next = data;
        is_there_new_data_next = (new_data_ack)? 1'b0 : is_there_new_data;
    end
end

always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        received_0xFE <= 1'b0;
        state_instuction <= 7'd0;
        is_there_new_instruct <= 1'b0;
        data <= 8'd0;
        is_there_new_data <= 1'b0;
    end
    else begin
        received_0xFE <= received_0xFE_next;
        state_instuction <= state_instuction_next;
        is_there_new_instruct <= is_there_new_instruct_next;
        data <= data_next;
        is_there_new_data <= is_there_new_data_next;
    end
end



// =========================================
//   The main finite state machine
// =========================================
// new_instr_ack is assigned 1 whenever state_instuction is used.
// new_data_ack is assigned 1 whenever state_instuction is used.
// For state transition, check the state instruction only at the states with "_idle" appended

/* These state parameters are defined above
parameter ST_IDLE                              = 7'h00;
parameter INSTRUCTION_ACK                      = 7'h01;  // not a state but just an acknowledge instruction
parameter ST_ERROR                             = 7'h0F;
parameter ST_UPDATE_RAM_get_num_of_frames      = 7'h10;  */
parameter ST_UPDATE_RAM_trigger                = 7'h11;
parameter ST_UPDATE_RAM_wait_first_data        = 7'h12;
parameter ST_UPDATE_RAM_first_idle             = 7'h13;  // waiting for instruction
parameter ST_UPDATE_RAM_wait_data              = 7'h14;
parameter ST_UPDATE_RAM_idle                   = 7'h15;  // waiting for instruction
parameter ST_UPDATE_RAM_finishing              = 7'h16;

reg [6:0]   total_frames, total_frames_next;
reg [25:0]  counter, counter_next, r_counter;
reg         fifo_wrreq;
reg [7:0]   fifo_wrdata;

always @ (*) begin
    case(states)
        ST_IDLE, ST_ERROR: begin
            if(is_there_new_instruct) begin
                new_instr_ack = 1'b1;
                new_data_ack  = 1'b0;
                states_next = state_instuction;
            end
            else begin
                new_instr_ack = 1'b0;
                new_data_ack  = 1'b1;  // clear all unwanted data since these states take instruction only
                states_next = states;
            end
        end

        /// ==== Updating the RAM ====================================
        ST_UPDATE_RAM_get_num_of_frames: begin
            new_instr_ack = 1'b0;
            new_data_ack  = 1'b1;
            states_next = ST_UPDATE_RAM_trigger;
            // update total_frames_next = data[5:0] + 1
            // update counter = 0
            // clear fifo
        end
        ST_UPDATE_RAM_trigger: begin
            new_instr_ack = 1'b0;
            new_data_ack  = 1'b0;
            states_next = ST_UPDATE_RAM_wait_first_data;
        end
        ST_UPDATE_RAM_wait_first_data: begin // for the first data because if total frames is 64, first data will be terminated
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            if(is_there_new_data)
                states_next = ST_UPDATE_RAM_idle;
            else
                states_next = ST_UPDATE_RAM_first_idle;
        end
        ST_UPDATE_RAM_first_idle: begin
            new_instr_ack = (is_there_new_instruct)? 1'b1 : 1'b0;
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == ST_IDLE)? ST_IDLE : ST_ERROR;
            else
                states_next = ST_UPDATE_RAM_wait_first_data;
        end
        ST_UPDATE_RAM_wait_data: begin
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            if(r_counter == {total_frames[5:0], 20'h0_0000})  // if total_frames[6:0] equals to 64, this line still give a correct ending.
                states_next = ST_UPDATE_RAM_finishing;
            else
                states_next = ST_UPDATE_RAM_idle;
        end
        ST_UPDATE_RAM_idle: begin
            new_instr_ack = (is_there_new_instruct)? 1'b1 : 1'b0;
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == ST_IDLE)? ST_IDLE : ST_ERROR;
            else
                states_next = ST_UPDATE_RAM_wait_data;
        end
        ST_UPDATE_RAM_finishing: begin
            new_instr_ack = (is_there_new_instruct)? 1'b1 : 1'b0;
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == INSTRUCTION_ACK)? ST_IDLE : ST_ERROR;
            else if(is_there_new_data)
                states_next = ST_ERROR;
            else
                states_next = states;
        end
        /// ==== End of Updating the RAM ====================================

        default: begin
            // TODO: temprorary dummies are put here and should be replaced
            states_next = states;
            new_data_ack = 1'b0;
            new_instr_ack = 1'b0;
        end
    endcase


    /// ==== Updating the RAM ====================================
    total_frames_next        = (states == ST_UPDATE_RAM_get_num_of_frames)? {1'b0,data[5:0]} + 1'b1 : total_frames;
    case(states)
        ST_UPDATE_RAM_get_num_of_frames:        counter_next = 26'd0;
        ST_UPDATE_RAM_trigger:                  counter_next = counter;
        ST_UPDATE_RAM_wait_first_data:          counter_next = (is_there_new_data)? counter + 1'b1: counter;
        ST_UPDATE_RAM_wait_data:                counter_next = (is_there_new_data)? counter + 1'b1: counter;
        ST_UPDATE_RAM_idle:                     counter_next = counter;
        default:                                counter_next = counter;
    endcase
    fifo_wrdata = data;
    fifo_wrreq = ((states == ST_UPDATE_RAM_wait_data || states == ST_UPDATE_RAM_wait_first_data) && is_there_new_data)? 1'b1 : 1'b0;
    /// ==== End of Updating the RAM ====================================
end


/// ==== Updating the RAM ====================================
// The following fifo stores the image data to be store into SDRAM.
// The special character 0xFE is the value of a pixel and does not
// have the meaning of the escaping character.
assign oNUM_IMAGES = total_frames;
assign oTRIGGER_WRITE_SDRAM = (states == ST_UPDATE_RAM_trigger);

fifo_sdram_write fifo_for_pixels(
    .aclr(iRST||(states==ST_UPDATE_RAM_get_num_of_frames)),
    .wrclk(iCLK),
    .wrreq(fifo_wrreq),
    .data(fifo_wrdata),
    .wrfull(),

    .rdclk(iDECODEDIMAGE_RDFIFO_CLK),
    .rdreq(iDECODEDIMAGE_RDFIFO_REQ),
    .q(oDECODEDIMAGE_RDFIFO_DATA),
    .rdempty(oDECODEDIMAGE_RDFIFO_EMPTY)
);
/// ==== End of Updating the RAM ====================================



// main sequential part
always @ (posedge iCLK) begin
    r_counter <= counter;
end
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states <= ST_IDLE;
        counter <= 0;
        total_frames <= 0;
    end
    else begin
        states <= states_next;
        counter <= counter_next;
        total_frames <= total_frames_next;
    end
end


assign oERROR = (states == ST_ERROR);


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
