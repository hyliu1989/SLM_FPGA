module jtag_uart_decode(
    input         iCLK,
    input         iSDRAM_CTRL_CLK,
    input         iRST,
    
    // jtag uart signals
    output        oJTAG_SLAVE_ADDR,
    output        oJTAG_SLAVE_RDREQ,
    input  [31:0] iJTAG_SLAVE_RDDATA,
    output        oJTAG_SLAVE_WRREQ,
    output [31:0] oJTAG_SLAVE_WRDATA,
    input         iJTAG_SLAVE_WAIT,
    
    // decoded signals (sync with iSDRAM_CTRL_CLK)
    input         iDECODEDIMAGE_RDFIFO_CLK,
    input         iDECODEDIMAGE_RDFIFO_REQ,
    output [7:0]  oDECODEDIMAGE_RDFIFO_DATA,
    output        oDECODEDIMAGE_RDFIFO_EMPTY,
    output        oTRIGGER_WRITE_SDRAM,
    output [5:0]  oSTARTING_FRAME,
    output [6:0]  oNUM_IMAGES_TO_DOWNLOAD,

    // decoded signals
    output [6:0]  oNUM_IMAGES_IN_MEM,
    output        oH_OFFSET_SIGN,
    output [7:0]  oH_OFFSET,
    output        oV_OFFSET_SIGN,
    output [7:0]  oV_OFFSET,
    
    output [15:0] oCYCLES_OF_DISPLAYING_EACH_IMAGE,
    
    output        oSEQUENCING_TRIGGER,
    output        oGALVE_SEQUENCING_TRIGGER,
    
    output [23:0] oGALVO_VALUES_X,
    output [23:0] oGALVO_VALUES_Y,
    
    output        oERROR,
    output [6:0]  oMONITORING_STATES
);

`include "jtag_decoder_param.h"


wire        is_idle_to_take_command;
reg         new_instr_ack, new_data_ack;
wire        is_there_new_instruct, is_there_new_data;
wire [6:0]  state_instuction;
wire [7:0]  data;

decode_raw_jtag_uart raw_0(
    .iCLK(iCLK),
    .iRST(iRST),

    // jtag uart signals
    .oJTAG_SLAVE_ADDR(oJTAG_SLAVE_ADDR),
    .oJTAG_SLAVE_RDREQ(oJTAG_SLAVE_RDREQ),
    .iJTAG_SLAVE_RDDATA(iJTAG_SLAVE_RDDATA),
    .oJTAG_SLAVE_WRREQ(oJTAG_SLAVE_WRREQ),
    .oJTAG_SLAVE_WRDATA(oJTAG_SLAVE_WRDATA),
    .iJTAG_SLAVE_WAIT(iJTAG_SLAVE_WAIT),

    .iDECODER_IDLE_TO_TAKE_COMMAND(is_idle_to_take_command),
    .iDECODER_ACK_INSTR(new_instr_ack),
    .iDECODER_ACK_DATA(new_data_ack),
    .oIS_THERE_NEW_INSTRUCTION(is_there_new_instruct),
    .oIS_THERE_NEW_DATA(is_there_new_data),
    .oSTATE_INSTRUCTION(state_instuction),
    .oDATA(data)
);



// =========================================
//   The main finite state machine
// =========================================
// new_instr_ack is assigned 1 whenever state_instuction is used.
// new_data_ack is assigned 1 whenever state_instuction is used.
// For state transition, check the state instruction only at the states with "_idle" appended
reg [6:0]   states, states_next, previous_states;

reg [5:0]   starting_frame, starting_frame_next;
reg [6:0]   total_frames_to_download, total_frames_to_download_next;
reg [6:0]   total_frames_in_mem, total_frames_in_mem_next;
reg [25:0]  counter, counter_next, r_counter;
reg         fifo_wrreq;
reg [7:0]   fifo_wrdata;

reg         update_horizontal, update_horizontal_next;
reg [7:0]   offset_h, offset_h_next, offset_v, offset_v_next;
reg         offset_sign_h, offset_sign_h_next, offset_sign_v, offset_sign_v_next;

reg [15:0]  cycles_of_display, cycles_of_display_next;

reg         update_galvo_horiz, update_galvo_horiz_next;
reg [23:0]  galvo_values_x, galvo_values_x_next, galvo_values_y, galvo_values_y_next;

assign is_idle_to_take_command = (states==ST_IDLE)||(states==ST_ERROR);

always @ (*) begin
    case(states)
        ST_IDLE, ST_ERROR, ST_WAIT_ACK, ST_LISTEN_TO_INTERRUPT:  new_instr_ack = (is_there_new_instruct)? 1'b1 : 1'b0;
        default:                                                 new_instr_ack = 1'b0;
    endcase

    case(states)
        ST_IDLE, ST_ERROR: begin
            if(is_there_new_instruct) begin
                new_data_ack  = 1'b0;
                states_next = state_instuction;
            end
            else begin
                new_data_ack  = 1'b1;  // clear all unwanted data since these states take instruction only
                states_next = states;
            end
        end
        
        ST_WAIT_ACK: begin
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == INSTRUCTION_ACK)? ST_IDLE : ST_ERROR;
            else if(is_there_new_data)
                states_next = ST_ERROR;
            else
                states_next = states;
        end
        
        ST_LISTEN_TO_INTERRUPT: begin
            new_data_ack = 1'b0;
            if(is_there_new_instruct)
                states_next = (state_instuction == ST_IDLE)? ST_IDLE : ST_ERROR;
            else
                states_next = previous_states;
        end

        
        /// ==== Updating the RAM ====================================
        ST_UPDATE_RAM_get_num_of_frames: begin
            new_data_ack  = 1'b1;
            states_next = ST_UPDATE_RAM_trigger;
            // update total_frames_to_download_next = data[5:0] + 1
            // update counter = 0
            // update starting_frame = 0
            // clear fifo
        end
        ST_UPDATE_RAM_trigger: begin
            new_data_ack  = 1'b0;
            states_next = ST_UPDATE_RAM_wait_first_data;
        end
        ST_UPDATE_RAM_wait_first_data: begin // for the first data because if total frames is 64, first data will be terminated
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_RAM_wait_data : ST_LISTEN_TO_INTERRUPT;
        end
        ST_UPDATE_RAM_wait_data: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            if(r_counter == {total_frames_to_download[5:0], 20'h0_0000})  // if total_frames_to_download[6:0] equals to 64, this line still give a correct ending.
                states_next = ST_WAIT_ACK;
            else
                states_next = ST_LISTEN_TO_INTERRUPT;
        end
        
        /// ==== Updating only a single frame part ====
        ST_UPDATE_RAM_SINGLE_get_frame_id: begin
            new_data_ack  = 1'b1;
            states_next = ST_UPDATE_RAM_trigger;
            // update total_frames_to_download_next = 1
            // starting_frame = data[5:0]
            // update single_counter = 0
            // clear fifo
        end
        /// ==== End of Updating the RAM ====================================


        /// ==== Updating total number of frames =========================
        ST_UPDATE_NUM_FRAMES_get_num: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_WAIT_ACK : ST_LISTEN_TO_INTERRUPT;
        end
        /// ==== End of Updating total number of frames =========================

        
        /// ==== Updating offsets ====================================
        ST_UPDATE_OFFSET_horizontal, ST_UPDATE_OFFSET_vertical: begin
            new_data_ack = 1'b0;
            states_next = ST_UPDATE_OFFSET_get_number;
            // update a register indicating which one of horizontal or vertical to update
        end
        ST_UPDATE_OFFSET_get_number: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_OFFSET_get_sign : ST_LISTEN_TO_INTERRUPT;
            // update the number
        end
        ST_UPDATE_OFFSET_get_sign: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_WAIT_ACK : ST_LISTEN_TO_INTERRUPT;
            // update the sign
        end
        /// ==== End of Updating Offsets ====================================
        
        
        /// ==== Updating Number of Displaying cycles ====================================
        ST_UPDATE_DISPLAY_CYC_get_num: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_DISPLAY_CYC_get_num_1 : ST_LISTEN_TO_INTERRUPT;
            // update the lower byte
        end
        ST_UPDATE_DISPLAY_CYC_get_num_1: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_WAIT_ACK : ST_LISTEN_TO_INTERRUPT;
            // update the upper byte
        end
        /// ==== End of Updating Number of Displaying cycles ====================================
        
        
        /// ==== Triggering the sequencing ====================================
        ST_START_SEQUENCE_trigger: begin
            new_data_ack = 1'b0;
            states_next = ST_WAIT_ACK;
        end
        ST_START_GALVO_SEQUENCE_trigger: begin
            new_data_ack = 1'b0;
            states_next = ST_WAIT_ACK;
        end
        /// ==== End of Triggering the sequencing ====================================
        
        
        /// ==== Updating Galvo information ====================================
        ST_UPDATE_GALVO_for_x, ST_UPDATE_GALVO_for_y: begin
            new_data_ack = 1'b0;
            states_next = ST_UPDATE_GALVO_value_0;
            // update a register indicating which one of horizontal or vertical to update
        end
        ST_UPDATE_GALVO_value_0: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_GALVO_value_1 : ST_LISTEN_TO_INTERRUPT;
            // update the number
        end
        ST_UPDATE_GALVO_value_1: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_UPDATE_GALVO_value_2 : ST_LISTEN_TO_INTERRUPT;
            // update the number
        end
        ST_UPDATE_GALVO_value_2: begin
            new_data_ack = (is_there_new_data)? 1'b1 : 1'b0;
            states_next = (is_there_new_data)? ST_WAIT_ACK : ST_LISTEN_TO_INTERRUPT;
            // update the number
        end
        /// ==== End of Updating Galvo information ====================================
        
        
        default: begin
            // TODO: temprorary dummies are put here and should be replaced
            new_data_ack = 1'b0;
            states_next = states;
        end
    endcase
end


/// ==== Updating the RAM ====================================
always @ (*) begin
    case(states)
        ST_UPDATE_RAM_get_num_of_frames:    total_frames_to_download_next = {1'b0,data[5:0]} + 1'b1;
        ST_UPDATE_RAM_SINGLE_get_frame_id:  total_frames_to_download_next = 7'd1;
        default:                            total_frames_to_download_next = total_frames_to_download;
    endcase

    case(states)
        ST_UPDATE_RAM_get_num_of_frames:    starting_frame_next = 6'd0;
        ST_UPDATE_RAM_SINGLE_get_frame_id:  starting_frame_next = data[5:0];
        default:                            starting_frame_next = starting_frame;
    endcase

    case(states)
        ST_UPDATE_RAM_get_num_of_frames:        counter_next = 26'd0;
        ST_UPDATE_RAM_trigger:                  counter_next = counter;
        ST_UPDATE_RAM_wait_first_data:          counter_next = 26'd1;  // directly assign the number even before capturing the data
        ST_UPDATE_RAM_wait_data:                counter_next = (is_there_new_data)? counter + 1'b1: counter;
        default:                                counter_next = counter;

        ST_UPDATE_RAM_SINGLE_get_frame_id:      counter_next = 26'd0;
    endcase

    fifo_wrdata = data;
    if(is_there_new_data)
        case(states)
            ST_UPDATE_RAM_wait_data, ST_UPDATE_RAM_wait_first_data:             fifo_wrreq = 1'b1; 
            default:                                                            fifo_wrreq = 1'b0;
        endcase
    else
        fifo_wrreq = 1'b0;
end

// The following fifo stores the image data to be store into SDRAM.
// The special character 0xFE in the fifo is the value of a pixel and
// does not have the meaning of the escaping character.
fifo_sdram_write fifo_for_pixels(
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

// Synchronizing signals
reg [1:0] m_trigger;
always @ (posedge iSDRAM_CTRL_CLK or posedge iRST) begin
    if(iRST) begin
        m_trigger <= 2'b00;
    end
    else begin
        if(m_trigger == 2'b00)
            m_trigger <= (states == ST_UPDATE_RAM_trigger)? 2'b01 : 2'b00;
        else
            m_trigger <= m_trigger + 1'b1;
    end
end

reg       o_trigger;
reg [5:0] o_starting_frame;
reg [6:0] o_num_to_download;
always @ (posedge iSDRAM_CTRL_CLK) begin
    if(m_trigger != 2'b00) begin
        o_starting_frame <= starting_frame;
        o_num_to_download <= total_frames_to_download;
    end
    if(m_trigger == 2'b10)
        o_trigger <= 1'b1;
    else
        o_trigger <= 1'b0;
end
assign oTRIGGER_WRITE_SDRAM = o_trigger;
assign oSTARTING_FRAME = o_starting_frame;
assign oNUM_IMAGES_TO_DOWNLOAD = o_num_to_download;


/// ==== End of Updating the RAM ====================================


/// ==== Updating total number of frames =========================
always @ (*) begin
    case(states)
        ST_UPDATE_RAM_get_num_of_frames:    total_frames_in_mem_next = {1'b0,data[5:0]} + 1'b1;
        ST_UPDATE_NUM_FRAMES_get_num:       total_frames_in_mem_next = (is_there_new_data)? data[6:0]: total_frames_in_mem;
        default:                            total_frames_in_mem_next = total_frames_in_mem;
    endcase
end
assign oNUM_IMAGES_IN_MEM = total_frames_in_mem;
/// ==== End of Updating total number of frames =========================


/// ==== Updating offsets ====================================
always @ (*) begin
    case(states)
        ST_UPDATE_OFFSET_horizontal: update_horizontal_next = 1'b1;
        ST_UPDATE_OFFSET_vertical:   update_horizontal_next = 1'b0;
        default:                     update_horizontal_next = update_horizontal;
    endcase
    // horizontal case
    if(update_horizontal && is_there_new_data) begin
        if(states == ST_UPDATE_OFFSET_get_number)
            offset_h_next = data;
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
    if(!update_horizontal && is_there_new_data) begin
        if(states == ST_UPDATE_OFFSET_get_number)
            offset_v_next = data;
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
    if(is_there_new_data)
        case(states)
            ST_UPDATE_DISPLAY_CYC_get_num:   cycles_of_display_next = {cycles_of_display[15:8], data};
            ST_UPDATE_DISPLAY_CYC_get_num_1: cycles_of_display_next = {data,  cycles_of_display[7:0]};
            default:                         cycles_of_display_next = cycles_of_display;
        endcase
    else
        cycles_of_display_next = cycles_of_display;
end
assign oCYCLES_OF_DISPLAYING_EACH_IMAGE = cycles_of_display;
/// ==== End of Updating Number of Displaying cycles ====================================


/// ==== Triggering the sequencing ====================================
assign oSEQUENCING_TRIGGER = (states == ST_START_SEQUENCE_trigger);
assign oGALVE_SEQUENCING_TRIGGER = (states == ST_START_GALVO_SEQUENCE_trigger);
/// ==== End of Triggering the sequencing ====================================


/// ==== Updating Galvo information ====================================
always @ (*) begin
    case(states)
        ST_UPDATE_GALVO_for_x: update_galvo_horiz_next = 1'b1;
        ST_UPDATE_GALVO_for_y: update_galvo_horiz_next = 1'b0;
        default:               update_galvo_horiz_next = update_galvo_horiz;
    endcase
    // horizontal case
    if(update_galvo_horiz && is_there_new_data)
        case(states)
            ST_UPDATE_GALVO_value_0: galvo_values_x_next = {galvo_values_x[23:16], galvo_values_x[15:8], data               };
            ST_UPDATE_GALVO_value_1: galvo_values_x_next = {galvo_values_x[23:16], data,                 galvo_values_x[7:0]};
            ST_UPDATE_GALVO_value_2: galvo_values_x_next = {data,                  galvo_values_x[15:8], galvo_values_x[7:0]};
            default:                 galvo_values_x_next = galvo_values_x;
        endcase
    else
        galvo_values_x_next = galvo_values_x;
    // vertical case
    if(!update_galvo_horiz && is_there_new_data)
        case(states)
            ST_UPDATE_GALVO_value_0: galvo_values_y_next = {galvo_values_y[23:16], galvo_values_y[15:8], data               };
            ST_UPDATE_GALVO_value_1: galvo_values_y_next = {galvo_values_y[23:16], data,                 galvo_values_y[7:0]};
            ST_UPDATE_GALVO_value_2: galvo_values_y_next = {data,                  galvo_values_y[15:8], galvo_values_y[7:0]};
            default:                 galvo_values_y_next = galvo_values_y;
        endcase
    else
        galvo_values_y_next = galvo_values_y;
end
assign oGALVO_VALUES_X = galvo_values_x;
assign oGALVO_VALUES_Y = galvo_values_y;
/// ==== End of Updating Galvo information ====================================
        

// main sequential part
always @ (posedge iCLK) begin
    r_counter <= counter;
    previous_states <= states;
end
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states <= ST_IDLE;
        starting_frame <= 0;
        total_frames_to_download <= 0;
        total_frames_in_mem <= 0;
        counter <= 0;
        update_horizontal <= 0;
        offset_h <= 8'd0;
        offset_sign_h <= 1'b0;
        offset_v <= 8'd0;
        offset_sign_v <= 1'b0;
        cycles_of_display <= 16'd0;
        update_galvo_horiz <= 1'b0;
        galvo_values_x <= 24'd0;
        galvo_values_y <= 24'd0;
    end
    else begin
        states <= states_next;
        starting_frame <= starting_frame_next;
        total_frames_to_download <= total_frames_to_download_next;
        total_frames_in_mem <= total_frames_in_mem_next;
        counter <= counter_next;
        update_horizontal <= update_horizontal_next;
        offset_h <= offset_h_next;
        offset_sign_h <= offset_sign_h_next;
        offset_v <= offset_v_next;
        offset_sign_v <= offset_sign_v_next;
        cycles_of_display <= cycles_of_display_next;
        update_galvo_horiz <= update_galvo_horiz_next;
        galvo_values_x <= galvo_values_x_next;
        galvo_values_y <= galvo_values_y_next;
    end
end

assign oERROR = (states == ST_ERROR);
assign oMONITORING_STATES = states;
endmodule
