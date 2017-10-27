// 1280 x 1024  (clock 108.0 MHz)
//	Horizontal Parameter	( Pixel )
parameter	H_FRONT	=	48;
parameter	H_SYNC	=	112;
parameter	H_BACK	=	248;
parameter	H_ACTIVE=	1280;	
parameter	H_TOTAL	=	H_FRONT + H_SYNC + H_BACK + H_ACTIVE;
//	Virtical Parameter		( Line )
parameter	V_FRONT=	1;
parameter	V_SYNC	=	3;	
parameter	V_BACK	=	38;
parameter	V_ACTIVE=	1024;
parameter	V_TOTAL	=	V_FRONT + V_SYNC + V_BACK + V_ACTIVE;
//	Start Offset
parameter	X_START	=	H_FRONT + H_SYNC + H_BACK;
parameter	Y_START	=	V_FRONT + V_SYNC + V_BACK;


/*
// 1920 x 1200  (clock 192.16 MHz)
//	Horizontal Parameter	( Pixel )
parameter	H_FRONT=	128;
parameter	H_SYNC	=	208;	
parameter	H_BACK	=	336;
parameter	H_ACTIVE	=	1920;
parameter	H_TOTAL=	2592;
//	Virtical Parameter		( Line )
parameter	V_FRONT=	1;
parameter	V_SYNC	=	3;
parameter	V_BACK	=	38;
parameter	V_ACTIVE	=	1200;
parameter	V_TOTAL=	1242;
//	Start Offset
parameter	X_START		=	H_SYNC+H_BACK;
parameter	Y_START		=	V_SYNC+V_BACK;
*/
