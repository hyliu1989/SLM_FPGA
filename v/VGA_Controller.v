module	VGA_Controller(	// FIFO read signal
						iData_R,
						iData_G,
						iData_B,
						oFIFO_RCLK,
						oFIFO_REQ,
						
						// FIFO load signal
						oFIFO_LOAD_REQ,
						oFIFO_LOAD_VLINE,
						oFIFO_CLEAR,
						
						//	VGA Side
						oVGA_R,
						oVGA_G,
						oVGA_B,
						oVGA_H_SYNC,
						oVGA_V_SYNC,
						oVGA_SYNC_N,
						oVGA_BLANK_N,
						oVGA_CLK,

						//	Control Signal
						iCLK,
						iRST_N
						//,iZOOM_MODE_SW
							);

`include "VGA_Param.h"

//	FIFO read
input		[7:0]	iData_R;
input		[7:0]	iData_G;
input		[7:0]	iData_B;
output		 		oFIFO_RCLK;
output	reg			oFIFO_REQ;
output	reg			oFIFO_LOAD_REQ;
output	reg	[12:0]	oFIFO_LOAD_VLINE;
output	reg			oFIFO_CLEAR;

//	VGA Side
output	reg	[7:0]	oVGA_R;
output	reg	[7:0]	oVGA_G;
output	reg	[7:0]	oVGA_B;
output	reg			oVGA_H_SYNC;
output	reg			oVGA_V_SYNC;
output	reg			oVGA_SYNC_N;
output	reg			oVGA_BLANK_N;
output				oVGA_CLK;
reg			[7:0]	mVGA_R;
reg			[7:0]	mVGA_G;
reg			[7:0]	mVGA_B;
reg					mVGA_H_SYNC;
reg					mVGA_V_SYNC;
wire				mVGA_SYNC_N;
reg					mVGA_BLANK_N;

//	Control Signal
input				iCLK;
input				iRST_N;

//	Internal Registers and Wires
reg		[12:0]		H_Cont;
reg		[12:0]		V_Cont;


////////////////////////////////////////////////////////

// FIFO signals
assign oFIFO_RCLK = ~iCLK;
always@(posedge iCLK or negedge iRST_N) begin
	if (!iRST_N) begin
		oFIFO_REQ <= 1'b0;
	end
	else begin
		// FIFO read request
		if ( H_Cont >= X_START-1 && H_Cont < X_START+H_ACTIVE-1 &&  // might need to be -2
			 V_Cont >= Y_START   && V_Cont < Y_START+V_ACTIVE ) begin
			oFIFO_REQ <= 1'b1;
		end
		else begin
			oFIFO_REQ <= 1'b0;
		end
	end
end
// FIFO load signals, no need to syncronize with read
always@(posedge iCLK or negedge iRST_N) begin
	if (!iRST_N) begin
		oFIFO_LOAD_VLINE <= 13'd0;
		oFIFO_LOAD_REQ <= 1'b0;
		oFIFO_CLEAR <= 1'b1;
	end
	else begin
		// the line to load is the next line of current (V_Cont-Y_START) line.
		if (V_Cont >= Y_START && V_Cont < Y_START+V_ACTIVE-1)
			oFIFO_LOAD_VLINE <= V_Cont-Y_START+1;
		else
			oFIFO_LOAD_VLINE <= 11'd0;
		// sending request to load a line
		if (V_Cont >= Y_START && V_Cont < Y_START+V_ACTIVE-1) begin
			if (H_Cont >= 2 && H_Cont < 4)
				oFIFO_LOAD_REQ <= 1'b1;
			else
				oFIFO_LOAD_REQ <= 1'b0;
		end
		else begin 
			if (V_Cont == 1 && H_Cont >= 2 && H_Cont < 4)
				oFIFO_LOAD_REQ <= 1'b1;
			else
				oFIFO_LOAD_REQ <= 1'b0;
		end
		// sending request to clear the fifo
		if (V_Cont == 0 && H_Cont >= 2 && H_Cont < 4)
			oFIFO_CLEAR <= 1'b1;
		else
			oFIFO_CLEAR <= 1'b0;
	end
end



// VGA signals
assign	oVGA_CLK	=	~iCLK;
assign	mVGA_SYNC_N	=	1'b0;

always@(posedge iCLK or negedge iRST_N) begin
	if(!iRST_N) begin
		mVGA_R			<=	0;
		mVGA_G			<=	0;
		mVGA_B			<=	0;
		mVGA_BLANK_N	<=	0;
	end
	else begin
		if ( H_Cont>=X_START && H_Cont<X_START+H_ACTIVE &&
			 V_Cont>=Y_START && V_Cont<Y_START+V_ACTIVE ) begin
			mVGA_R <= iData_R;
			mVGA_G <= iData_G;
			mVGA_B <= iData_B;
			mVGA_BLANK_N <= 1'b1;
		end
		else begin
			mVGA_R <= 8'd0;
			mVGA_G <= 8'd0;
			mVGA_B <= 8'd0;
			mVGA_BLANK_N <= 1'b0;
		end
	end
end

//	H_Sync Generator, Ref. 25.175 MHz Clock
parameter SYNC_ACTIVE = 1'b1;  // active high
parameter SYNC_DEACT  = 1'b0;
always@(posedge iCLK or negedge iRST_N)
begin
	if(!iRST_N) begin
		H_Cont		<=	0;
		mVGA_H_SYNC	<=	SYNC_DEACT;
	end
	else begin
		//	H_Sync Counter
		if( H_Cont < H_TOTAL-1 )
			H_Cont	<=	H_Cont+1'b1;
		else
			H_Cont	<=	0;
		
		//	H_Sync Generator
		if( H_Cont == 0 + H_FRONT )
			mVGA_H_SYNC	<=	SYNC_ACTIVE;
		else if ( H_Cont == H_SYNC + H_FRONT )
			mVGA_H_SYNC	<=	SYNC_DEACT;
	end
end

//	V_Sync Generator, Ref. H_Sync
always@(posedge iCLK or negedge iRST_N)
begin
	if(!iRST_N)
	begin
		V_Cont		<=	0;
		mVGA_V_SYNC	<=	SYNC_DEACT;
	end
	else begin
		//	When H_Sync Re-start
		if(H_Cont == 0) begin
			//	V_Sync Counter
			if( V_Cont < V_TOTAL-1 )
				V_Cont	<=	V_Cont+1'b1;
			else
				V_Cont	<=	0;
			
			//	V_Sync Generator
			if( V_Cont == 0 + V_FRONT - 1 )
				mVGA_V_SYNC	<=	SYNC_ACTIVE;
			else if ( V_Cont == V_SYNC + V_FRONT - 1 )
				mVGA_V_SYNC	<=	SYNC_DEACT;
		end
	end
end


always@(posedge iCLK or negedge iRST_N)
	begin
		if (!iRST_N)
			begin
				oVGA_R <= 0;
				oVGA_G <= 0;
                oVGA_B <= 0;
				oVGA_BLANK_N <= 0;
				oVGA_SYNC_N <= 0;
				oVGA_H_SYNC <= 0;
				oVGA_V_SYNC <= 0;
			end
		else
			begin
				oVGA_R <= mVGA_R;
				oVGA_G <= mVGA_G;
                oVGA_B <= mVGA_B;
				oVGA_BLANK_N <= mVGA_BLANK_N;
				oVGA_SYNC_N <= mVGA_SYNC_N;
				oVGA_H_SYNC <= mVGA_H_SYNC;
				oVGA_V_SYNC <= mVGA_V_SYNC;				
			end               
	end

endmodule