module idexpipe(
    
    CLOCK,RESET,
    //
    ID_EXinRegDst,ID_EXinMemRead,ID_EXinMemtoReg,ID_EXinMemWrite,ID_EXinALUSrc,ID_EXinRegWrite,
    ID_EXinjalsig,
    ID_EXinIALUCtl,
    ID_EXinlwusig,
    ID_EXinSIZE,
    /*from ctrlmux(CTRL),10 lines*/
    ID_EXinjalrsig,//11
    ID_EXinALUctl,//12
    ID_EXinshamtsig,//13
    ID_EXinmainshamtsig,//14
    ID_EXinToandlinkorder,//15
    ID_EXinbalandlink,//16
    ID_EXinfromC,ID_EXinfromD,//17,18
    ID_EXinImm,//19
    ID_EXintoEXRs,ID_EXintoEXRt,ID_EXintoEXRd,//20,21,22

    //---------------------------------

    ID_EXoutRegDst,ID_EXoutMemRead,ID_EXoutMemtoReg,ID_EXoutMemWrite,
    ID_EXoutALUSrc,ID_EXoutRegWrite,
    ID_EXoutjalsig,
    ID_EXoutIALUCtl,
    ID_EXoutlwusig,
    ID_EXoutSIZE,
    
    ID_EXoutjalrsig,//11
    ID_EXoutALUctl,//12
    ID_EXoutshamtsig,//13
    ID_EXoutmainshamt,//14
    ID_EXoutToandlinkorder,//15
    ID_EXoutbalandlink,//16
    ID_EXoutfromC,ID_EXoutfromD,//17,18
    ID_EXoutImm,//19
    ID_EXouttoEXRs,ID_EXouttoEXRt,ID_EXouttoEXRd,//20,21,22

    
);



    input CLOCK,RESET;
    //
    input ID_EXinRegDst,ID_EXinMemRead,ID_EXinMemtoReg,ID_EXinMemWrite,ID_EXinALUSrc,ID_EXinRegWrite;
    input ID_EXinjalsig;
    input [3:0] ID_EXinIALUCtl;
    input ID_EXinlwusig;
    input [1:0] ID_EXinSIZE;
    /*from ctrlmux(CTRL),10 lines*/
    input ID_EXinjalrsig;//11
    input [3:0] ID_EXinALUctl;//12
    input ID_EXinshamtsig;//13
    input [4:0]ID_EXinmainshamtsig;//14
    input [31:0] ID_EXinToandlinkorder;//15
    input  ID_EXinbalandlink;//16
    input [31:0] ID_EXinfromC,ID_EXinfromD;//17,18
    input [31:0] ID_EXinImm;//19
    input [4:0] ID_EXintoEXRs,ID_EXintoEXRt,ID_EXintoEXRd;//20,21,22

    //---------------------------------

    output reg ID_EXoutRegDst,ID_EXoutMemRead,ID_EXoutMemtoReg,ID_EXoutMemWrite;
    output reg ID_EXoutALUSrc,ID_EXoutRegWrite;
    output reg ID_EXoutjalsig;
    output reg [3:0] ID_EXoutIALUCtl;
    output reg ID_EXoutlwusig;
    output reg [1:0]ID_EXoutSIZE;
    //10
    output reg ID_EXoutjalrsig;//11
    output reg [3:0] ID_EXoutALUctl;//12
    output reg ID_EXoutshamtsig;//13
    output reg [4:0] ID_EXoutmainshamt;//14
    output reg [31:0] ID_EXoutToandlinkorder;//15//<---PC******
    output reg ID_EXoutbalandlink;//16
    output reg[31:0] ID_EXoutfromC,ID_EXoutfromD;//17,18
    output reg [31:0] ID_EXoutImm;//19
    output reg [4:0] ID_EXouttoEXRs,ID_EXouttoEXRt,ID_EXouttoEXRd;//20,21,22



    always@(posedge CLOCK or negedge RESET) begin
        if(RESET == 1'b0) begin
            ID_EXoutRegDst <= 0;
            ID_EXoutMemRead<= 0;
            ID_EXoutMemtoReg<=0;
            ID_EXoutMemWrite<=0;
            ID_EXoutALUSrc<=0;
            ID_EXoutRegWrite<=0;
            ID_EXoutjalsig<=0;
            ID_EXoutIALUCtl<=0;
            ID_EXoutlwusig<=0;
            ID_EXoutSIZE <=0;
            //10
            ID_EXoutjalrsig<=0;//11
            ID_EXoutALUctl<=0;//12
            ID_EXoutshamtsig<=0;//13
            ID_EXoutmainshamt<=0;//14
            ID_EXoutToandlinkorder<=0;//15
            ID_EXoutbalandlink<=0;//16
            ID_EXoutfromC<=0;
            ID_EXoutfromD<=0;//17,18
            ID_EXoutImm<=0;//19
            ID_EXouttoEXRs<=0;
            ID_EXouttoEXRt<=0;
            ID_EXouttoEXRd<=0;//20,21,22
        end
        else if(CLOCK == 1'b1) begin
            ID_EXoutRegDst <= ID_EXinRegDst;
            ID_EXoutMemRead<= ID_EXinMemRead;
            ID_EXoutMemtoReg<=ID_EXinMemtoReg;
            ID_EXoutMemWrite<=ID_EXinMemWrite;
            ID_EXoutALUSrc<=ID_EXinALUSrc;
            ID_EXoutRegWrite<=ID_EXinRegWrite;
            ID_EXoutjalsig<=ID_EXinjalsig;
            ID_EXoutIALUCtl<=ID_EXinIALUCtl;
            ID_EXoutlwusig<=ID_EXinlwusig;
            ID_EXoutSIZE <=ID_EXinSIZE;
            //10
            ID_EXoutjalrsig<=ID_EXinjalrsig;//11
            ID_EXoutALUctl<=ID_EXinALUctl;//12
            ID_EXoutshamtsig<=ID_EXinshamtsig;//13
            ID_EXoutmainshamt<=ID_EXinmainshamtsig;//14
            ID_EXoutToandlinkorder<=ID_EXinToandlinkorder;//15
            ID_EXoutbalandlink<=ID_EXinbalandlink;//16
            ID_EXoutfromC<=ID_EXinfromC;
            ID_EXoutfromD<=ID_EXinfromD;//17,18
            ID_EXoutImm<=ID_EXinImm;//19
            ID_EXouttoEXRs<=ID_EXintoEXRs;
            ID_EXouttoEXRt<=ID_EXintoEXRt;
            ID_EXouttoEXRd<=ID_EXintoEXRd;//20,21,22
        end
    end
endmodule