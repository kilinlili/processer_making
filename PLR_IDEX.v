module idexpipe(
    input CLOCK,RESET,
    //
    input inRegDst,inMemRead,inMemtoReg,inMemWrite,inALUSrc,inRegWrite,
    input injalsig,
    input [3:0] inIALUCtl,
    input inlwusig,
    input [1:0] inSIZE,
    /*from ctrlmux(CTRL),10 lines*/
    input injalrsig,//11
    input [3:0] inALUctl,//12
    input inshamtsig,//13
    input [4:0]inmainshamt,//14
    input [31:0] inToandlinkorder,//15
    input  inbalandlink,//16
    input [31:0] infromC,infromD,//17,18
    input [31:0] inImm,//19
    input [4:0] intoEXRs,intoEXRt,intoEXRd,//20,21,22

    //---------------------------------

    output reg IFIDoutRegDst,IFIDoutMemRead,IFIDoutMemtoReg,IFIDoutMemWrite,
    output reg IFIDoutALUSrc,IFIDoutRegWrite,
    output reg IFIDoutjalsig,
    output reg [3:0] IFIDoutIALUCtl,
    output reg IFIDoutlwusig,
    output reg [1:0]IFIDoutSIZE,
    //10
    output reg IFIDoutjalrsig,//11
    output reg [3:0] IFIDoutALUctl,//12
    output reg IFIDoutshamtsig,//13
    output reg [4:0] IFIDoutmainshamt,//14
    output reg [31:0] IFIDoutToandlinkorder,//15
    output  reg IFIDoutbalandlink,//16
    output reg[31:0] IFIDoutfromC,IFIDoutfromD,//17,18
    output reg [31:0] IFIDoutImm,//19
    output reg [4:0] IFIDouttoEXRs,IFIDouttoEXRt,IFIDouttoEXRd//20,21,22
    
);

    always@(posedge CLOCK or negedge RESET) begin
        if(RESET == 1'b0) begin
            IFIDoutRegDst <= 0;
            IFIDoutMemRead<= 0;
            IFIDoutMemtoReg<=0;
            IFIDoutMemWrite<=0;
            IFIDoutALUSrc<=0;
            IFIDoutRegWrite<=0;
            IFIDoutjalsig<=0;
            IFIDoutIALUCtl<=0;
            IFIDoutlwusig<=0;
            IFIDoutSIZE <=0;
            //10
            IFIDoutjalrsig<=0;//11
            IFIDoutALUctl<=0;//12
            IFIDoutshamtsig<=0;//13
            IFIDoutmainshamt<=0;//14
            IFIDoutToandlinkorder<=0;//15
            IFIDoutbalandlink<=0;//16
            IFIDoutfromC<=0;
            IFIDoutfromD<=0;//17,18
            IFIDoutImm<=0;//19
            IFIDouttoEXRs<=0;
            IFIDouttoEXRt<=0;
            IFIDouttoEXRd<=0;//20,21,22
        end
        else if(CLOCK == 1'b1) begin
            IFIDoutRegDst <= inRegDst;
            IFIDoutMemRead<= inMemRead;
            IFIDoutMemtoReg<=inMemtoReg;
            IFIDoutMemWrite<=inMemWrite;
            IFIDoutALUSrc<=inALUSrc;
            IFIDoutRegWrite<=inRegWrite;
            IFIDoutjalsig<=injalsig;
            IFIDoutIALUCtl<=inIALUCtl;
            IFIDoutlwusig<=inlwusig;
            IFIDoutSIZE <=inSIZE;
            //10
            IFIDoutjalrsig<=injalrsig;//11
            IFIDoutALUctl<=inALUctl;//12
            IFIDoutshamtsig<=inshamtsig;//13
            IFIDoutmainshamt<=inmainshamt;//14
            IFIDoutToandlinkorder<=inToandlinkorder;//15
            IFIDoutbalandlink<=inbalandlink;//16
            IFIDoutfromC<=infromC;
            IFIDoutfromD<=infromD;//17,18
            IFIDoutImm<=inImm;//19
            IFIDouttoEXRs<=intoEXRs;
            IFIDouttoEXRt<=intoEXRt;
            IFIDouttoEXRd<=intoEXRd;//20,21,22
        end
    end
endmodule