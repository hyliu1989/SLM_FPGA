module decode_raw_jtag_uart(
    input   iCLK,
    input   iRST,

    // jtag uart signals
    output        oJTAG_SLAVE_ADDR,
    output        oJTAG_SLAVE_RDREQ,
    input  [31:0] iJTAG_SLAVE_RDDATA,
    output        oJTAG_SLAVE_WRREQ,
    output [31:0] oJTAG_SLAVE_WRDATA,
    input         iJTAG_SLAVE_WAIT,

    input   iDECODER_IDLE_TO_TAKE_COMMAND,
    input   iDECODER_ACK_INSTR,
    input   iDECODER_ACK_DATA,
    output  oIS_THERE_NEW_INSTRUCTION,
    output  oIS_THERE_NEW_DATA,
    output [6:0]  oSTATE_INSTRUCTION,
    output [7:0]  oDATA
);

`include "jtag_decoder_param.h"

// ====================================================
//   Communicating with JTAG-UART core
// ====================================================
wire [7:0]  r0_data_or_cmd;
wire        r0_data_or_cmd_valid;
reg  [7:0]  data_or_cmd, r1_data_or_cmd;
reg         data_or_cmd_valid, r1_data_or_cmd_valid;

uart_avalon_extraction  extraction_0(
    .iCLK(iCLK),
    .iRST(iRST),
    .oJTAG_SLAVE_ADDR(oJTAG_SLAVE_ADDR),
    .oJTAG_SLAVE_RDREQ(oJTAG_SLAVE_RDREQ),
    .iJTAG_SLAVE_RDDATA(iJTAG_SLAVE_RDDATA),
    .oJTAG_SLAVE_WRREQ(oJTAG_SLAVE_WRREQ),
    .oJTAG_SLAVE_WRDATA(oJTAG_SLAVE_WRDATA),
    .iJTAG_SLAVE_WAIT(iJTAG_SLAVE_WAIT),
    .oDATA_TO_PARSE(r0_data_or_cmd),
    .oDATA_TO_PARSE_VALID(r0_data_or_cmd_valid)
);
always @ (posedge iCLK) begin
    r1_data_or_cmd <= r0_data_or_cmd;
    r1_data_or_cmd_valid <= r0_data_or_cmd_valid;
    data_or_cmd <= r1_data_or_cmd;
    data_or_cmd_valid <= r1_data_or_cmd_valid;
end





// =============================================================
//   Data parsing to extract state instruction and actual data
// =============================================================
wire [1:0]  local_states;
reg         received_0xFE, received_0xFE_next;
reg [6:0]   state_instuction, state_instuction_next;  // an instruction to states, `state_instuction` will be check as `states` progresses.
reg         is_there_new_instruct, is_there_new_instruct_next;
reg [7:0]   data, data_next;
reg         is_there_new_data, is_there_new_data_next;

assign local_states = {received_0xFE, iDECODER_IDLE_TO_TAKE_COMMAND};
parameter ESCAPING         = 2'b1?;
parameter WHEN_OUTER_STATE_IS_IDLE = 2'b01;
parameter WHEN_OUTER_STATE_IS_PROC = 2'b00;

always @ (*) begin
    // received_0xFE_next
    if(data_or_cmd_valid) begin
        if(received_0xFE)
            received_0xFE_next = 1'b0;
        else
            received_0xFE_next = (data_or_cmd == HOSTCMD_ESCAPE);
    end
    else
        received_0xFE_next = received_0xFE;
    
    
    // state_instuction_next, is_there_new_instruct_next, data_next, is_there_new_data_next
    if(data_or_cmd_valid) begin
        casez(local_states)
            /// 0xFE interruption behavior
            ESCAPING: begin
                // For instruction to the states
                case(data_or_cmd)
                    ESCAPECMD_RETURN_TO_IDLE: begin
                        state_instuction_next = ST_IDLE;
                        is_there_new_instruct_next = 1'b1;
                    end
                    ESCAPECMD_ACKNOWLEDGE: begin
                        state_instuction_next = INSTRUCTION_ACK;
                        is_there_new_instruct_next = 1'b1;
                    end
                    ESCAPECMD_ESCAPE_CHAR: begin
                        state_instuction_next = 7'bxxx_xxxx;
                        is_there_new_instruct_next = 1'b0;  // no new state to move to
                    end
                    default: begin
                        state_instuction_next = ST_ERROR;
                        is_there_new_instruct_next = 1'b1;
                    end
                endcase
                
                // For data, the only data after a 0xFE escaping
                data_next              = (data_or_cmd == ESCAPECMD_ESCAPE_CHAR)? HOSTCMD_ESCAPE : 8'bxxxx_xxxx;
                is_there_new_data_next = (data_or_cmd == ESCAPECMD_ESCAPE_CHAR)? 1'b1 : 1'b0;
            end

            /// When outer state is Idle or Idle with Error, it takes instructions
            WHEN_OUTER_STATE_IS_IDLE: begin
                data_next = data_or_cmd;

                casez(data_or_cmd)
                    HOSTCMD_SEND_IMAGES:  is_there_new_data_next = 1'b1;  // the data in this case is the number of frames to transfer minus 1.
                    HOSTCMD_SEND_SINGLE:  is_there_new_data_next = 1'b1;  // the data in this case is the frames id (0~63 inclusive).
                    default:              is_there_new_data_next = 1'b0;  // usually there is no data in data_or_cmd when taking commands
                endcase

                case(data_or_cmd)
                    HOSTCMD_ESCAPE:       is_there_new_instruct_next = 1'b0;
                    default:              is_there_new_instruct_next = 1'b1;
                endcase

                casez(data_or_cmd)
                    HOSTCMD_SEND_IMAGES:            state_instuction_next = ST_UPDATE_RAM_get_num_of_frames;  // Kick off transferring data to SDRAM
                    HOSTCMD_SEND_SINGLE:            state_instuction_next = ST_UPDATE_RAM_SINGLE_get_frame_id;  // Kick off transferring single image data to the SDRAM
                    HOSTCMD_UPDATE_OFFSET_H:        state_instuction_next = ST_UPDATE_OFFSET_horizontal;
                    HOSTCMD_UPDATE_OFFSET_V:        state_instuction_next = ST_UPDATE_OFFSET_vertical;
                    HOSTCMD_UPDATE_CYC_DISPLAY:     state_instuction_next = ST_UPDATE_DISPLAY_CYC_get_num;
                    HOSTCMD_TRIGGER_SEQUENCING:     state_instuction_next = ST_START_SEQUENCE_trigger;
                    HOSTCMD_TRIGGER_GALVO_SEQUENCE: state_instuction_next = ST_START_GALVO_SEQUENCE_trigger;
                    HOSTCMD_UPDATE_GALVO_NUM_POS:   state_instuction_next = ST_UPDATE_GALVO_NUM_POS_get_num_0;
                    HOSTCMD_UPDATE_TOT_NUM_FRAMES:  state_instuction_next = ST_UPDATE_NUM_FRAMES_get_num;
                    HOSTCMD_UPDATE_STATIC_DISPLAY:  state_instuction_next = ST_UPDATE_STATIC_DISPLAY_get_id;
                    HOSTCMD_ESCAPE:                 state_instuction_next = 7'bxxx_xxxx;  // Bypass the escaping
                    default:                        state_instuction_next = ST_ERROR;  // otherwise, go to the error state
                endcase
            end

            /// When machine is in some processing, no instruction but obtain the data.
            WHEN_OUTER_STATE_IS_PROC: begin
                data_next = data_or_cmd;
                is_there_new_data_next = (data_or_cmd == HOSTCMD_ESCAPE)? 1'b0 : 1'b1;

                state_instuction_next = 7'bxxx_xxxx;
                is_there_new_instruct_next = 1'b0;
            end
        endcase
    end
    // when data_or_cmd is not valid, mostly hold the positions of the 4 registers
    else begin
        state_instuction_next = state_instuction;
        is_there_new_instruct_next = (iDECODER_ACK_INSTR)? 1'b0 : is_there_new_instruct;
        
        data_next = data;
        is_there_new_data_next = (iDECODER_ACK_DATA)? 1'b0 : is_there_new_data;
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
assign oIS_THERE_NEW_INSTRUCTION = is_there_new_instruct;
assign oIS_THERE_NEW_DATA = is_there_new_data;
assign oSTATE_INSTRUCTION = state_instuction;
assign oDATA = data;
endmodule
