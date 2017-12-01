// Commands used in PC
parameter HOSTCMD_ESCAPE                    = 8'hFE;

parameter HOSTCMD_SEND_IMAGES               = 8'b10??_????;  // Update memory
parameter HOSTCMD_SEND_SINGLE               = 8'b01??_????;  // Update memory of one image
parameter HOSTCMD_UPDATE_OFFSET_H           = 8'h01;  // Update offsets
parameter HOSTCMD_UPDATE_OFFSET_V           = 8'h02;  // Update offsets
parameter HOSTCMD_UPDATE_CYC_DISPLAY        = 8'h03;  // Update number of displaying cycles for each image
parameter HOSTCMD_TRIGGER_SEQUENCING        = 8'h04;  // Triggering the sequencing
parameter HOSTCMD_TRIGGER_GALVO_SEQUENCE    = 8'h05;  // Triggering the sequencing
parameter HOSTCMD_UPDATE_GALVO_NUM_POS      = 8'h06;  // Update number of positions of Galvo
parameter HOSTCMD_UPDATE_TOT_NUM_FRAMES     = 8'h07;  // Update total number of frames in memory
parameter HOSTCMD_UPDATE_STATIC_DISPLAY     = 8'h08;  // Update static display image id



// ===== Commands and states on FPGA side ===========================
/// Commands
// Commands after the Escape character
parameter ESCAPECMD_RETURN_TO_IDLE             = 8'h00;
parameter ESCAPECMD_ACKNOWLEDGE                = 8'h01;
parameter ESCAPECMD_ESCAPE_CHAR                = 8'hFE;

/// States
// System states
parameter ST_IDLE                              = 7'h0_0;
parameter INSTRUCTION_ACK                      = 7'h1_0;  // not a state but just an acknowledge instruction
parameter ST_LISTEN_TO_INTERRUPT               = 7'h2_0;  // waiting for instruction
parameter ST_WAIT_ACK                          = 7'h6_0;
parameter ST_ERROR                             = 7'h7_0;

// Update memory
parameter ST_UPDATE_RAM_get_num_of_frames      = 7'h0_1;
parameter ST_UPDATE_RAM_trigger                = 7'h1_1;
parameter ST_UPDATE_RAM_wait_first_data        = 7'h2_1;
parameter ST_UPDATE_RAM_wait_data              = 7'h3_1;
// Update memory of one image
parameter ST_UPDATE_RAM_SINGLE_get_frame_id    = 7'h7_1;

// Update offsets
parameter ST_UPDATE_OFFSET_horizontal          = 7'h0_2;
parameter ST_UPDATE_OFFSET_vertical            = 7'h1_2;
parameter ST_UPDATE_OFFSET_get_number          = 7'h2_2;
parameter ST_UPDATE_OFFSET_get_sign            = 7'h3_2;

// Update number of displaying cycles for each image
parameter ST_UPDATE_DISPLAY_CYC_get_num        = 7'h0_3;
parameter ST_UPDATE_DISPLAY_CYC_get_num_1      = 7'h1_3;

// Triggering the sequencing
parameter ST_START_SEQUENCE_trigger            = 7'h0_4;
parameter ST_START_GALVO_SEQUENCE_trigger      = 7'h1_4;

// Update number of positions of Galvo
parameter ST_UPDATE_GALVO_NUM_POS_get_num_0    = 7'h0_5;
parameter ST_UPDATE_GALVO_NUM_POS_get_num_1    = 7'h1_5;
parameter ST_UPDATE_GALVO_NUM_POS_get_num_2    = 7'h2_5;
parameter ST_UPDATE_GALVO_NUM_POS_get_num_3    = 7'h3_5;

// Update total number of frames in memory
parameter ST_UPDATE_NUM_FRAMES_get_num         = 7'h0_6;

// Update static display image id
parameter ST_UPDATE_STATIC_DISPLAY_get_id      = 7'h0_7;

