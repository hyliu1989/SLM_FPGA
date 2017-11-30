module sequencer(
    input         iCLK,
    input         iRST,

    // settings
    input [7:0]   iCAMERA_TRIGGER_MILLISEC,
    input [7:0]   iGALVO_TRIGGER_MILLISEC,
    input [6:0]   iNUM_SLM_IMAGES,
    input [15:0]  iCYCLES_OF_DISPLAY_FOR_EACH_IMAGE,
    input [31:0]  iNUM_OF_GALVO_POSITIONS,

    // signals
    input         iTRIG_WITHOUT_GALVO,
    input         iTRIG_WITH_GALVO,
    output        oCAMERA_TRIGGER,
    output        oGALVO_CHANGE_TRIGGER,
    input         iGALVO_ACK,
    input         iVGA_FRAME_SYNC,
    output [5:0]  oCURRENT_DISPLAY_FRAME_ID,
    output        oBUSY
);

reg [7:0]  dejitter;
wire       vga_frame_sync;
reg [1:0]  vga_frame_sync_edge_detect;
wire       vga_frame_sync_neg_edge;
assign vga_frame_sync = |dejitter;
assign vga_frame_sync_neg_edge = (vga_frame_sync_edge_detect == 2'b10);
always @ (posedge iCLK) begin
    dejitter <= {dejitter[6:0], iVGA_FRAME_SYNC};
    vga_frame_sync_edge_detect <= {vga_frame_sync_edge_detect[0], vga_frame_sync};
end


parameter ST_IDLE                             = 5'd0;
parameter ST_RECORD_SETTING                   = 5'd1;
parameter ST_SIGNAL_GALVO                     = 5'd2;
parameter ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_1 = 5'd3;
parameter ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_2 = 5'd4;
parameter ST_WAIT_FOR_VGA_REFRESH_1           = 5'd5;
parameter ST_WAIT_FOR_VGA_REFRESH_2           = 5'd6;
parameter ST_SIGNAL_CAMERA                    = 5'd7;
parameter ST_WAIT_A_VGA_CYCLE                 = 5'd8;                      
parameter ST_COUNT_VGA_CYCLES_0               = 5'd16;
parameter ST_COUNT_VGA_CYCLES_1               = 5'd17;
parameter ST_COUNT_VGA_CYCLES_2               = 5'd18;
parameter ST_CHECK_LAST_FRAME                 = 5'd9;
parameter ST_CHECK_LAST_GALVO_POSITION_0      = 5'd10;
parameter ST_CHECK_LAST_GALVO_POSITION_1      = 5'd11;
parameter ST_CHECK_LAST_GALVO_POSITION_2      = 5'd12;
parameter ST_CHECK_LAST_GALVO_POSITION_3      = 5'd13;
parameter ST_CHECK_LAST_GALVO_POSITION_4      = 5'd14;
parameter ST_CHECK_LAST_GALVO_POSITION_5      = 5'd15;

reg  [4:0]  states;
reg         is_with_galvo;
reg  [31:0] num_total_galvo_positions;
reg  [15:0] num_cycles_per_image;
reg  [6:0]  num_total_slm_images;
reg  [7:0]  cam_trigger_counter;
reg  [7:0]  galvo_trigger_counter;

reg         galvo_ack_catch, second_sync_negedge_catch;
reg  [31:0] galvo_cnt;
reg  [31:0] galvo_cnt_plus_one_at_state[5:1];
reg         is_galvo_cnt_ended_at_state[5:1];
integer i;
reg  [15:0] cycle_cnt;
reg  [15:0] cycle_cnt_plus_one_at_state[2:1];
reg         is_last_cycle_at_state[2:1];
reg  [6:0]  img_cnt;
wire        is_last_image;

// states, is_with_galvo
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        states <= ST_IDLE;
        is_with_galvo <= 1'b0;
    end
    else begin
        case(states)
            ST_IDLE:                             states <= (iTRIG_WITH_GALVO||iTRIG_WITHOUT_GALVO)? ST_RECORD_SETTING : states;
            ST_RECORD_SETTING:                   states <= (is_with_galvo)? ST_SIGNAL_GALVO : ST_WAIT_FOR_VGA_REFRESH_1;
            ST_SIGNAL_GALVO:                     states <= ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_1;
            ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_1: states <= (vga_frame_sync_neg_edge)? ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_2 : states;
            ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_2: states <= (galvo_ack_catch&&second_sync_negedge_catch)? ST_SIGNAL_CAMERA : states;
            ST_WAIT_FOR_VGA_REFRESH_1:           states <= (vga_frame_sync_neg_edge)? ST_WAIT_FOR_VGA_REFRESH_2 : states;
            ST_WAIT_FOR_VGA_REFRESH_2:           states <= (vga_frame_sync_neg_edge)? ST_SIGNAL_CAMERA : states;
            ST_SIGNAL_CAMERA:                    states <= ST_WAIT_A_VGA_CYCLE;
            ST_WAIT_A_VGA_CYCLE:                 states <= (vga_frame_sync)? ST_COUNT_VGA_CYCLES_0 : states;
            ST_COUNT_VGA_CYCLES_0:               states <= ST_COUNT_VGA_CYCLES_1;
            ST_COUNT_VGA_CYCLES_1:               states <= ST_COUNT_VGA_CYCLES_2;
            ST_COUNT_VGA_CYCLES_2:               states <= (is_last_cycle_at_state[2])? ST_CHECK_LAST_FRAME : ST_WAIT_A_VGA_CYCLE;
            ST_CHECK_LAST_FRAME: begin
                if(is_last_image) begin
                    if(is_with_galvo)            states <= ST_CHECK_LAST_GALVO_POSITION_0;
                    else                         states <= ST_IDLE;
                end 
                else                             states <= ST_WAIT_FOR_VGA_REFRESH_1;
            end
            ST_CHECK_LAST_GALVO_POSITION_0:      states <= ST_CHECK_LAST_GALVO_POSITION_1;
            ST_CHECK_LAST_GALVO_POSITION_1:      states <= ST_CHECK_LAST_GALVO_POSITION_2;
            ST_CHECK_LAST_GALVO_POSITION_2:      states <= ST_CHECK_LAST_GALVO_POSITION_3;
            ST_CHECK_LAST_GALVO_POSITION_3:      states <= ST_CHECK_LAST_GALVO_POSITION_4;
            ST_CHECK_LAST_GALVO_POSITION_4:      states <= ST_CHECK_LAST_GALVO_POSITION_5;
            ST_CHECK_LAST_GALVO_POSITION_5:      states <= (is_galvo_cnt_ended_at_state[5])? ST_IDLE : ST_SIGNAL_GALVO;
        endcase

        if(states == ST_IDLE) begin
            is_with_galvo <= iTRIG_WITH_GALVO;
        end
        else begin
            is_with_galvo <= is_with_galvo;
        end
    end
end

// num_total_galvo_positions, num_total_slm_images, num_cycles_per_image
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        num_total_galvo_positions <= 0;
        num_total_slm_images <= 0;
        num_cycles_per_image <= 0; 
    end
    else begin
        if(states == ST_RECORD_SETTING) begin
            num_total_galvo_positions <= iNUM_OF_GALVO_POSITIONS;
            num_total_slm_images <= iNUM_SLM_IMAGES;
            num_cycles_per_image <= iCYCLES_OF_DISPLAY_FOR_EACH_IMAGE;
        end
    end
end

// galvo_ack_catch, second_sync_negedge_catch
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        galvo_ack_catch <= 1'b0;
        second_sync_negedge_catch <= 1'b0;
    end
    else begin
        case(states)
            ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_1: begin
                galvo_ack_catch <= iGALVO_ACK || galvo_ack_catch;
                second_sync_negedge_catch <= 1'b0;
            end
            ST_WAIT_FOR_GALVO_AND_VGA_REFRESH_2: begin
                galvo_ack_catch <= iGALVO_ACK || galvo_ack_catch;
                second_sync_negedge_catch <= second_sync_negedge_catch || vga_frame_sync_neg_edge;
            end
            default: begin
                galvo_ack_catch <= 1'b0;
                second_sync_negedge_catch <= 1'b0;
            end
        endcase
    end
end

// galvo_cnt, galvo_cnt_plus_one_at_state, is_galvo_cnt_ended_at_state
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        galvo_cnt <= 0;
        for(i = 1; i <= 5; i = i + 1) begin
            galvo_cnt_plus_one_at_state[i] <= 0;
            is_galvo_cnt_ended_at_state[i] <= 0;
        end
    end
    else begin
        // galvo_cnt
        case(states)
            ST_IDLE:                        galvo_cnt <= 0;
            ST_CHECK_LAST_GALVO_POSITION_5: galvo_cnt <= galvo_cnt_plus_one_at_state[5];
            default:                        galvo_cnt <= galvo_cnt;
        endcase
        // galvo_cnt_plus_one_at_state[1], is_galvo_cnt_ended_at_state[1]
        if(states == ST_CHECK_LAST_GALVO_POSITION_0) begin
            galvo_cnt_plus_one_at_state[1] <= galvo_cnt + 1'b1;  // value at state 1 (1 as in ST_CHECK_LAST_GALVO_POSITION_1)
            is_galvo_cnt_ended_at_state[1] <= ((galvo_cnt+1'b1) == num_total_galvo_positions); // value at state 1
        end
        else begin
            galvo_cnt_plus_one_at_state[1] <= galvo_cnt_plus_one_at_state[1];
            is_galvo_cnt_ended_at_state[1] <= is_galvo_cnt_ended_at_state[1];
        end
        // // galvo_cnt_plus_one_at_state[5:2], is_galvo_cnt_ended_at_state[5:2]
        for(i = 2; i <= 5; i = i + 1) begin
            galvo_cnt_plus_one_at_state[i] <= galvo_cnt_plus_one_at_state[i-1];
            is_galvo_cnt_ended_at_state[i] <= is_galvo_cnt_ended_at_state[i-1];
        end
    end
end

// cycle_cnt, cycle_cnt_plus_one_at_state, is_last_cycle_at_state
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        cycle_cnt <= 0;
        for(i = 1; i <= 2; i = i + 1) begin
            cycle_cnt_plus_one_at_state[i] <= 0;
            is_last_cycle_at_state[i] <= 0;
        end
    end
    else begin
        // cycle_cnt
        case(states)
            ST_IDLE:               cycle_cnt <= 0;
            ST_COUNT_VGA_CYCLES_2: cycle_cnt <= cycle_cnt_plus_one_at_state[2];
            default:               cycle_cnt <= cycle_cnt;
        endcase
        // cycle_cnt_plus_one_at_state[1], is_last_cycle_at_state[1]
        if(states == ST_COUNT_VGA_CYCLES_0) begin
            cycle_cnt_plus_one_at_state[1] <= cycle_cnt + 1'b1;  // value at state 1 (1 as in ST_COUNT_VGA_CYCLES_1)
            is_last_cycle_at_state[1] <= ((cycle_cnt+1'b1) == num_cycles_per_image); // value at state 1
        end
        else begin
            cycle_cnt_plus_one_at_state[1] <= cycle_cnt_plus_one_at_state[1];
            is_last_cycle_at_state[1] <= is_last_cycle_at_state[1];
        end
        // // cycle_cnt_plus_one_at_state[2:2], is_last_cycle_at_state[2:2]
        for(i = 2; i <= 2; i = i + 1) begin
            cycle_cnt_plus_one_at_state[i] <= cycle_cnt_plus_one_at_state[i-1];
            is_last_cycle_at_state[i] <= is_last_cycle_at_state[i-1];
        end
    end
end

// img_cnt, is_last_image
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        img_cnt <= 0;
    end
    else begin
        if(states == ST_IDLE)
            img_cnt <= 0;
        if(states == ST_CHECK_LAST_FRAME)
            img_cnt <= img_cnt + 1'b1;
        else
            img_cnt <= img_cnt;
    end
end
assign is_last_image = ((img_cnt + 1'b1) == num_total_slm_images);


// cam_trigger_counter, galvo_trigger_counter
millisec_counter cam_trigger(
    .iCLOCK50(iCLK),
    .iRST(iRST),
    .iTRIGGER(states == ST_SIGNAL_CAMERA),
    .iMILLISEC_TO_COUNT(iCAMERA_TRIGGER_MILLISEC),  // [7:0]
    .oPULSE(oCAMERA_TRIGGER)  // pulse of length specified by iMILLISEC_TO_COUNT
);

millisec_counter galvo_trigger(
    .iCLOCK50(iCLK),
    .iRST(iRST),
    .iTRIGGER(states == ST_SIGNAL_GALVO),
    .iMILLISEC_TO_COUNT(iGALVO_TRIGGER_MILLISEC),  // [7:0]
    .oPULSE(oGALVO_CHANGE_TRIGGER)  // pulse of length specified by iMILLISEC_TO_COUNT
);

assign oBUSY = (states != ST_IDLE);
assign oCURRENT_DISPLAY_FRAME_ID = img_cnt[5:0];

/*
always @ (posedge iCLK or posedge iRST) begin
    if(iRST) begin
        
    end
    else begin
        
    end
end
*/



endmodule
