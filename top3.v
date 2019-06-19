module top3(
    /*IFstage*/
    IAD,
    IDT,// --->ifmodule's "IDTORD"
    ACKI_n,
    /*IFstage*/
    /*MEMstage*/
    DAD,
    DDT,//------------------------->input & output 
    MREQ,
    WRITE,
    SIZE,
    ACKD_n,
    /*MEMstage*/
    /*warikomi*/
    rst,
    OINT_n,
    IACK_n,
    /*warikomi*/clk 
);

/*IFstage*/
    output [31:0] IAD;
    input [31:0] IDT;// --->ifmodule's "IDTORD"
    input ACKI_n;
    /*IFstage*/
    /*MEMstage*/
    output [31:0] DAD;
    inout [31:0] DDT;//------------------------->input & output 
    output MREQ;
    output WRITE;
    output [1:0] SIZE;
    input ACKD_n;
    /*MEMstage*/
    /*warikomi*/
    input rst;
    input [2:0] OINT_n;
    output IACK_n;
    /*warikomi*/
    input clk;

//-----------------------------------------------------------------------------------
    wire [31:0] TOADD;






//--------------------------------------------------------------------------------

    ifidpipe i_ifidpipe(
    /*input*/
    .CLOCK(),.RESET(),
    .IFIDWRITE,//flush1
    .FROMIFPC4(),
    .FROMORDER(),
    .IFFLASH(),
    /*input*/
    /*output*/
    .TOADD(TOADD),.TOMAINORDER()
    /*output*/
    );



    id1 i_id1(
        .CLOCK(),.RESET(),
        .IFID_ORDER(),
        .IFID_PCADD4(TOADD),
        .ID_IN_MEMID_IN_WBREGWRITE(),
        .ID_IN_MEMWBRegisterRt(),
        .ID_IN_WBdata(),
        .ID_IN_MEMdata(),
        .ID_IN_IDEXMEMREAD(),
        .ID_IN_IDEXREGWRITE(),
        .ID_IN_IDEXREGISTERRT(),
        .ID_IN_IDEXREGISTERRD(),
        .ID_IN_EXMEMREGWRITE(),
        .ID_IN_EXMEMMEMREAD(),
        .ID_IN_EXMEMREGISTERRDRT(),
        .ID_IN_WRITEADD(),
        .ID_IN_DATAIN32(),
        .ID_IN_WBREGWRITE(),

        .ID_OUT_IFIDWRITE(),
        .ID_OUT_PCWRITE(),
        .ID_OUT_toandlinkorder(),
        .ID_OUT_jumpaddress(),
        .ID_OUT_toEXRd(),
        .ID_OUT_toEXRs(),
        .ID_OUT_toEXRt(),
        .ID_OUT_toshamt(),
        .ID_OUT_toandlinkorder(),
        .ID_OUT_jumpaddress(),
        .ID_OUT_toEXRd(),
        .ID_OUT_toEXRs(),
        .ID_OUT_toEXRt(),
        .ID_OUT_Immout(),
        .ID_OUT_toshamt(),
        .ID_OUT_DATA1(),
        .ID_OUT_DATA2(),
        .ID_OUT_baddress(),
        .ID_OUT_balandlink(),
        .ID_OUT_beqtojrjalr32(),
        .ID_OUT_toIFpcsrc(),
        .ID_OUT_toIFjump(),
        .ID_OUT_ALUctltopipe(),
        .ID_OUT_ALUopshamtsig(),
        .ID_OUT_ALUopjalrsig(),
        .ID_OUT_ALUoprjump(),
        .ID_OUT_RegDst(),
        .ID_OUT_MemRead(),
        .ID_OUT_MemtoReg(),
        .ID_OUT_MemWrite(),
        .ID_OUT_ALUSrc(),
        .ID_OUT_RegWrite(),
        .ID_OUT_jalsig(),
        .ID_OUT_IALUCtl(),
        .ID_OUT_lwusig(),
        .ID_OUT_SIZE()
    );




endmodule