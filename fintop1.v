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




endmodule