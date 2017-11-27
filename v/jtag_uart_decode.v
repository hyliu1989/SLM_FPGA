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
    
    output        oH_OFFSET_SIGN,
    output [7:0]  oH_OFFSET,
    output        oV_OFFSET_SIGN,
    output [7:0]  oV_OFFSET,
    
    output [15:0] oCYCLES_OF_DISPLAYING_EACH_IMAGE,
    
    output        oERROR,
    output [6:0]  oMONITORING_STATES
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


reg [6:0]   states, states_next, previous_states;
reg         new_instr_ack, new_data_ack;
parameter ST_IDLE                              = 7'h0_0;
parameter INSTRUCTION_ACK                      = 7'h1_0;  // not a state but just an acknowledge instruction
parameter ST_LISTEN_TO_INTERRUPT               = 7'h2_0;
parameter ST_WAIT_ACK                          = 7'h6_0;
parameter ST_ERROR                             = 7'h7_0;

parameter ST_UPDATE_RAM_get_num_of_frames      = 7'h0_1;

parameter ST_UPDATE_OFFSET_horizontal          = 7'h0_2;
parameter ST_UPDATE_OFFSET_vertical            = 7'h1_2;

parameter ST_UPDATE_DISPLAY_CYC_get_num        = 7'h0_3;



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

                    // Kick off updating horizontal offset
                    8'h01: begin
                        state_instuction_next = ST_UPDATE_OFFSET_horizontal;
                        is_there_new_instruct_next = 1'b1;
                        is_there_new_data_next = 1'b0;
                    end
                    
                    // Kick off updating vertical offset
                    8'h02: begin
                        state_instuction_next = ST_UPDATE_OFFSET_vertical;
                        is_there_new_instruct_next = 1'b1;
                        is_there_new_data_next = 1'b0;
                    end
                    
                    // Kick off updating displaying cycles
                    8'h03: begin
                        state_instuction_next = ST_UPDATE_DISPLAY_CYC_get_num;
                        is_there_new_instruct_next = 1'b1;
                        is_there_new_data_next = 1'b0;
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
                if(data_or_cmd == 8'hFE) begin
                    state_instuction_next = 7'bxxx_xxxx;
                    is_there_new_instruct_next = 1'b0;
                    is_there_new_data_next = 1'b0;
                end
                else begin
                    state_instuction_next = 7'bxxx_xxxx;
                    is_there_new_instruct_next = 1'b0;
                    is_there_new_data_next = 1'b1;
                end
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
parameter ST_IDLE                              = 7'h0_0;
parameter INSTRUCTION_ACK                      = 7'h1_0;  // not a state but just an acknowledge instruction
parameter ST_LISTEN_TO_INTERRUPT               = 7'h2_0;  // waiting for instruction
parameter ST_WAIT_ACK                          = 7'h6_0;
parameter ST_ERROR                             = 7'h7_0;  */

/* Update memory 
parameter ST_UPDATE_RAM_get_num_of_frames      = 7'h0_1;  */
parameter ST_UPDATE_RAM_trigger                = 7'h1_1;
parameter ST_UPDATE_RAM_wait_first_data        = 7'h2_1;
parameter ST_UPDATE_RAM_wait_data              = 7'h3_1;

/* Update offsets
parameter ST_UPDATE_OFFSET_horizontal          = 7'h0_2;
parameter ST_UPDATE_OFFSET_vertical            = 7'h1_2;  */
parameter ST_UPDATE_OFFSET_get_number          = 7'h2_2;
parameter ST_UPDATE_OFFSET_get_sign            = 7'h3_2;

/* Update number of displaying cycles for each image
parameter ST_UPDATE_DISPLAY_CYC_get_num        = 7'h0_3;  */
parameter ST_UPDATE_DISPLAY_CYC_get_num_1      = 7'h1_3;


reg [6:0]   total_frames, total_frames_next;
reg [25:0]  counter, counter_next, r_counter;
reg         fifo_wrreq;
reg [7:0]   fifo_wrdata;

reg         update_horizontal, update_horizontal_next;
reg [7:0]   offset_h, offset_h_next, offset_v, offset_v_next;
reg         offset_sign_h, offset_sign_h_next, offset_sign_v, offset_sign_v_next;

reg [15:0]  cycles_of_display, cycles_of_display_next;

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
        
        ST_WAIT_ACK: begin
            new_instr_ack = (is_there_new_instruct)? 1'b1 : 1'b0;
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == INSTRUCTION_ACK)? ST_IDLE : ST_ERROR;
            else if(is_there_new_data)
                states_next = ST_ERROR;
            else
                states_next = states;
        end
        
        ST_LISTEN_TO_INTERRUPT: begin
            new_instr_ack = (is_there_new_instruct)? 1'b1 : 1'b0;
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == ST_IDLE)? ST_IDLE : ST_ERROR;
            else
                states_next = previous_states;
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
                states_next = ST_UPDATE_RAM_wait_data;
            else
                states_next = ST_LISTEN_TO_INTERRUPT;
        end
        ST_UPDATE_RAM_wait_data: begin
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            if(r_counter == {total_frames[5:0], 20'h0_0000})  // if total_frames[6:0] equals to 64, this line still give a correct ending.
                states_next = ST_WAIT_ACK;
            else
                states_next = ST_LISTEN_TO_INTERRUPT;
        end
        /// ==== End of Updating the RAM ====================================

        
        /// ==== Updating offsets ====================================
        ST_UPDATE_OFFSET_horizontal, ST_UPDATE_OFFSET_vertical: begin
            new_instr_ack = 1'b0;
            new_data_ack = 1'b0;
            states_next = ST_UPDATE_OFFSET_get_number;
            // update a register indicating which on of horizontal or vertical to update
        end
        ST_UPDATE_OFFSET_get_number: begin
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_OFFSET_get_sign : ST_LISTEN_TO_INTERRUPT;
            // update the number
        end
        ST_UPDATE_OFFSET_get_sign: begin
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_WAIT_ACK : ST_LISTEN_TO_INTERRUPT;
            // update the sign
        end
        /// ==== End of Updating Offsets ====================================
        
        
        /// ==== Updating Number of Displaying cycles ====================================
        ST_UPDATE_DISPLAY_CYC_get_num: begin
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_DISPLAY_CYC_get_num_1 : ST_LISTEN_TO_INTERRUPT;
            // update the lower byte
        end
        ST_UPDATE_DISPLAY_CYC_get_num_1: begin
            new_instr_ack = 1'b0;
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_WAIT_ACK : ST_LISTEN_TO_INTERRUPT;
            // update the upper byte
        end
        /// ==== End of Updating Number of Displaying cycles ====================================
        
        default: begin
            // TODO: temprorary dummies are put here and should be replaced
            states_next = states;
            new_data_ack = 1'b0;
            new_instr_ack = 1'b0;
        end
    endcase
end


/// ==== Updating the RAM ====================================
always @ (*) begin
    total_frames_next        = (states == ST_UPDATE_RAM_get_num_of_frames)? {1'b0,data[5:0]} + 1'b1 : total_frames;
    case(states)
        ST_UPDATE_RAM_get_num_of_frames:        counter_next = 26'd0;
        ST_UPDATE_RAM_trigger:                  counter_next = counter;
        ST_UPDATE_RAM_wait_first_data:          counter_next = 26'd1;  // directly assign the number even before capturing the data
        ST_UPDATE_RAM_wait_data:                counter_next = (is_there_new_data)? counter + 1'b1: counter;
        default:                                counter_next = counter;
    endcase
    fifo_wrdata = data;
    fifo_wrreq = ((states == ST_UPDATE_RAM_wait_data || states == ST_UPDATE_RAM_wait_first_data) && is_there_new_data)? 1'b1 : 1'b0;
end

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


/// ==== Updating offsets ====================================
always @ (*) begin
    case(states)
        ST_UPDATE_OFFSET_horizontal: update_horizontal_next = 1'b1;
        ST_UPDATE_OFFSET_vertical:   update_horizontal_next = 1'b0;
        default:                     update_horizontal_next = update_horizontal;
    endcase
    // horizontal case
    if(update_horizontal) begin
        if(states == ST_UPDATE_OFFSET_get_number)
            offset_h_next = (is_there_new_data)? data: offset_h;
        else
            offset_h_next = offset_h;
        if(states == ST_UPDATE_OFFSET_get_sign)
            offset_sign_h_next = data[0];
        else
            offset_sign_h_next = offset_sign_h;
    end
    else begin
        offset_h_next = offset_h;
        offset_sign_h_next = offset_sign_h;
    end
    // vertical case
    if(!update_horizontal) begin
        if(states == ST_UPDATE_OFFSET_get_number)
            offset_v_next = (is_there_new_data)? data: offset_v;
        else
            offset_v_next = offset_v;
        if(states == ST_UPDATE_OFFSET_get_sign)
            offset_sign_v_next = data[0];
        else
            offset_sign_v_next = offset_sign_v;
    end
    else begin
        offset_v_next = offset_v;
        offset_sign_v_next = offset_sign_v;
    end
end
assign oH_OFFSET = (offset_h[7] == 1'b1)? 8'd128 : offset_h;
assign oH_OFFSET_SIGN = offset_sign_h;
assign oV_OFFSET = (offset_v[7] == 1'b1)? 8'd128 : offset_v;
assign oV_OFFSET_SIGN = offset_sign_v;
/// ==== End of Updating Offsets ====================================


/// ==== Updating Number of Displaying cycles ====================================
always @ (*) begin
    case(states)
        ST_UPDATE_DISPLAY_CYC_get_num:
            cycles_of_display_next = (is_there_new_data)? {cycles_of_display[15:8], data} : cycles_of_display;
        ST_UPDATE_DISPLAY_CYC_get_num_1:
            cycles_of_display_next = (is_there_new_data)? {data,  cycles_of_display[7:0]} : cycles_of_display;
        default: 
            cycles_of_display_next = cycles_of_display;
    endcase
end
assign oCYCLES_OF_DISPLAYING_EACH_IMAGE = cycles_of_display;
/// ==== End of Updating Number of Displaying cycles ====================================


// main sequential part
always @ (posedge iCLK) begin
    r_counter <= counter;
end
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states <= ST_IDLE;
        previous_states <= ST_IDLE;
        counter <= 0;
        total_frames <= 0;
        update_horizontal <= 0;
        offset_h <= 8'd0;
        offset_sign_h <= 1'b0;
        offset_v <= 8'd0;
        offset_sign_v <= 1'b0;
        cycles_of_display <= 16'd0;
    end
    else begin
        states <= states_next;
        previous_states <= states;
        counter <= counter_next;
        total_frames <= total_frames_next;
        update_horizontal <= update_horizontal_next;
        offset_h <= offset_h_next;
        offset_sign_h <= offset_sign_h_next;
        offset_v <= offset_v_next;
        offset_sign_v <= offset_sign_v_next;
        cycles_of_display <= cycles_of_display_next;
    end
end

assign oERROR = (states == ST_ERROR);
assign oMONITORING_STATES = states;
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
