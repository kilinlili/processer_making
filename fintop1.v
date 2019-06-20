module maintop(
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
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

    wire IF_inWRITEPC;
    wire [1:0] IF_infromJUMP,IF_infromRJUMP;
    wire [31:0] IF_infromJJAL,IF_infromJRJALR,IF_inBRANCHGO;
    wire IF_infromPCSRC;

    wire IF_outFLASHIF;
    wire [31:0]IF_outgoifidpc4;
//-----------------------------------------------------------------------------------
//IFIDPIPE
    wire IF_IDinpipeWRITE;//<--harzard
    //in
    wire [31:0] IF_IDoutTOADD,IF_IDoutTOMAINORDER;//-->IDstage
    //out
//-----------------------------------------------------------------------------------
//IDSTAGE
    wire ID_inMEMWBREGWRITE;//<--MEMWBpipe
    wire [4:0] ID_inMEMWBRegisterRt;
    wire [31:0] ID_inWBdata,ID_inMEMdata;

    wire ID_inIDEXMEMREAD,ID_inIDEXREGWRITE;
    wire [4:0]ID_inIDEXREGISTERRT,ID_inIDEXREGISTERRD;
    wire ID_inEXMEMREGWRITE,ID_inEXMEMMEMREAD;
    wire [4:0]ID_inEXMEMREGISTERRDRT;
    wire [31:0]ID_inDATAIN32;
    //in
    //insum 17

    wire ID_outbalandlink,ID_outALUopjalrsig,ID_outALUopshamtsig;
    wire [31:0]ID_outtoandlinkorder;
    wire [4:0] ID_outtoEXRd,ID_outtoEXRs,ID_outtoEXRt,ID_outtoshamt;
    wire [31:0]ID_outImmout;
    wire [3:0]ID_outALUctltopipe;
    wire [31:0]ID_outDATA1,ID_outDATA2;
    //out1
    wire ID_outRegDst,ID_outMemRead,ID_outMemtoReg,ID_outMemWrite,ID_outALUSrc,ID_outRegWrite;//6
    wire ID_outjalsig,ID_outlwusig;
    wire [3:0] ID_outIALUCtl;
    wire [1:0] ID_outSIZE;
    //out2
    //outsum22

//-----------------------------------------------------------------------------------
//IDEXPIPE
    //insum 0

    wire ID_EXoutRegDst,ID_EXoutMemtoReg,ID_EXoutALUSrc,ID_EXoutMemWrite;
    wire ID_EXoutjalsig,ID_EXoutlwusig;
    wire [3:0] ID_EXoutIALUCtl;
    wire [1:0] ID_EXoutSIZE;
    //out1
    wire ID_EXoutjalrsig;//1bits
    wire [3:0] ID_EXoutALUctl;//4bits
    wire ID_EXoutshamtsig;//1bits 
    wire [4:0] ID_EXoutmainshamt;//5bits
    wire [31:0] ID_EXoutToandlinkorder;//32bits//PC******************
    wire ID_EXoutbalandlink;//1bits
    wire [31:0] ID_EXoutfromC,ID_EXoutfromD,ID_EXoutImm;//32bits 
    wire [4:0] ID_EXouttoEXRs;//5bits 
    //out2 
//-----------------------------------------------------------------------------------
//EXSTAGE
    //insum 0

    wire [31:0] EX_outanswer,EX_outfboutpipe;
    wire EX_outtoANDLINK;
    wire [4:0] EX_outtopipereg5;
    //out 

//-----------------------------------------------------------------------------------
//EXMEMPIPE
    wire EX_MEMoutMemtoReg,EX_MEMoutMemWrite,EX_MEMoutlwusig,EX_MEMoutANDLINK;
    wire [1:0] EX_MEMoutSIZE;
    wire [31:0] EX_MEMoutPCadd,EX_MEMoutforb;
    //out

//-----------------------------------------------------------------------------------
//MEMSTAGE

//-----------------------------------------------------------------------------------
//WBSTAGE
    wire MEM_WBoutMEMTOREG,MEM_WBoutLWSIG;
    wire [1:0] MEM_WBoutSIZE;
    wire [31:0] MEM_WBoutlwans,MEM_WBoutPC,MEM_WBoutRform,MEM_WBoutandlinlsig;

//-----------------------------------------------------------------------------------




//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------






    if1 userifstage(
    .CLOCK(clk),.RESET(rst),//1bits
    .IF_inWRITEPC(IF_inWRITEPC),//1bits<---
    .IF_infromJUMP(IF_infromJUMP),.IF_infromRJUMP(IF_infromRJUMP),//2bits<--ID!
    .IF_infromJJAL(IF_infromJJAL),.IF_infromJRJALR(IF_infromJRJALR),//32bits<--ID!
    .IF_infromPCSRC(IF_infromPCSRC),//1bit<--ID
    .IF_inBRANCHGO(IF_inBRANCHGO),//32bits<--ID

    .IF_outIADADD(IAD), //------------------------to top output//
    .IF_outFLASHIF(IF_outFLASHIF),//-------------------------->to pipeline //1bit
    .IF_outgoifidpc4(IF_outgoifidpc4) //-------------->to pipeline //32bits
    );

    ifidpipe userpipeifid(
    /*input*/
    .CLOCK(clk),.RESET(rst),
    .IF_IDinpipeWRITE(IF_IDinpipeWRITE),//1bits//stoll
    .IF_IDinPC4(IF_outgoifidpc4),//32bits
    .IF_IDinORDER(IDT),//32bits
    .IF_IDinFLASH(IF_outFLASHIF),//1bits
    /*input*/
    /*output*/
    .IF_IDoutTOADD(IF_IDoutTOADD),//-->IDstage
    .IF_IDoutTOMAINORDER(IF_IDoutTOMAINORDER)//-->IDstage 
    /*output*/
    );

    id1 useridstage(
    .CLOCK(clk),.RESET(clk),
    .IFID_ORDER(IF_IDoutTOMAINORDER),
    .IFID_PCADD4(IF_IDoutTOADD),
    //4
    .ID_inMEMWBREGWRITE(ID_inMEMWBREGWRITE),//1bits
    .ID_inMEMWBRegisterRt(ID_inMEMWBRegisterRt),//5bits
    .ID_inWBdata(ID_inWBdata),.ID_inMEMdata(ID_inMEMdata),//32bits
    .ID_inIDEXMEMREAD(ID_inIDEXMEMREAD),.ID_inIDEXREGWRITE(ID_inIDEXREGWRITE),//1bits
    .ID_inIDEXREGISTERRT(ID_inIDEXREGISTERRT),.ID_inIDEXREGISTERRD(ID_inIDEXREGISTERRD),//5bits
    .ID_inEXMEMREGWRITE(ID_inEXMEMREGWRITE),.ID_inEXMEMMEMREAD(ID_inEXMEMMEMREAD),//1bits
    .ID_inEXMEMREGISTERRDRT(ID_inEXMEMREGISTERRDRT),//5bits
    //5bits <--register locate?//check1
    .ID_inDATAIN32(ID_inDATAIN32),//32bits <-- register in data//check1
    //13
    //input sum is 17

    .ID_outIFIDWRITE(IF_IDinpipeWRITE),.ID_outPCWRITE(IF_inWRITEPC),//-->IFIDPIPE! & IF!
    .ID_outtoandlinkorder(ID_outtoandlinkorder),//32//--> pipe
    .ID_outjumpaddress(IF_infromJJAL),//32//-->IF!
    //4
    .ID_outtoEXRd(ID_outtoEXRd),//5bits-->pipe
    .ID_outtoEXRs(ID_outtoEXRs),//5bits-->pipe
    .ID_outtoEXRt(ID_outtoEXRt),//5bits-->pipe
    .ID_outImmout(ID_outImmout),//32bits-->pipe
    .ID_outtoshamt(ID_outtoshamt),//5bits -->pipe--------------------------------
    .ID_outDATA1(ID_outDATA1),.ID_outDATA2(ID_outDATA2),//32bits//-->pipe 
    .ID_outbaddress(IF_inBRANCHGO),//32bits-->IF!
    .ID_outbalandlink(ID_outbalandlink),//1bits-->pipe
    .ID_outbeqtojrjalr32(IF_infromJRJALR),//32bits-->IF!
    //////shamtsig ga nai!!!!!!!
    //10
    .ID_outtoIFpcsrc(IF_infromPCSRC),//1bits-->IF!
    .ID_outtoIFjump(IF_infromJUMP),//2bits-->IF!
    .ID_outALUctltopipe(ID_outALUctltopipe),//4bits-->pipe
    .ID_outALUopshamtsig(ID_outALUopshamtsig),//1bits-->pipe----------------
    .ID_outALUopjalrsig(ID_outALUopjalrsig),//1bits-->pipe
    .ID_outALUoprjump(IF_infromRJUMP),//2bits-->IF!
    //6

    .ID_outRegDst(ID_outRegDst),.ID_outMemRead(ID_outMemRead),.ID_outMemtoReg(ID_outMemtoReg),
    .ID_outMemWrite(ID_outMemWrite),.ID_outALUSrc(ID_outALUSrc),.ID_outRegWrite(ID_outRegWrite),
    .ID_outjalsig(ID_outjalsig),//1
    .ID_outIALUCtl(ID_outIALUCtl),//4
    .ID_outlwusig(ID_outlwusig),//1
    .ID_outSIZE(ID_outSIZE)//2
    //10-->allpipe
    //output sum is 30
    );//ok

    idexpipe userpipeidex(
    
    .CLOCK(clk),.RESET(rst),
    //
    .ID_EXinRegDst(ID_outRegDst),.ID_EXinMemRead(ID_outMemRead),.ID_EXinMemtoReg(ID_outMemtoReg),.ID_EXinMemWrite(ID_outMemWrite),
    .ID_EXinALUSrc(ID_outALUSrc),.ID_EXinRegWrite(ID_outRegWrite),
    .ID_EXinjalsig(ID_outjalsig),//1bits
    .ID_EXinIALUCtl(ID_outIALUCtl),//4bits
    .ID_EXinlwusig(ID_outlwusig),//1bits
    .ID_EXinSIZE(ID_outSIZE),//2bits
    /*from ctrlmux(CTRL)(),10 lines*/

    .ID_EXinjalrsig(ID_outALUopjalrsig),//1bit<--ID
    .ID_EXinALUctl(ID_outIALUCtl),//4bit<--ID
    .ID_EXinshamtsig(ID_outALUopshamtsig),//1bits 
    .ID_EXinmainshamtsig(ID_outtoshamt),//5bit******
    .ID_EXinToandlinkorder(ID_outtoandlinkorder),//32bits
    .ID_EXinbalandlink(ID_outbalandlink),//1bits
    .ID_EXinfromC(ID_outDATA1),.ID_EXinfromD(ID_outDATA2),//32bits
    .ID_EXinImm(ID_outImmout),//32bits
    .ID_EXintoEXRs(ID_outtoEXRs),.ID_EXintoEXRt(ID_outtoEXRt),.ID_EXintoEXRd(ID_outtoEXRd),//20(),21(),22
    //input sum 22
    //---------------------------------

    .ID_EXoutRegDst(ID_EXoutRegDst),.ID_EXoutMemRead(ID_inIDEXMEMREAD),//-->EX ///-->pipe & ID!!
    .ID_EXoutMemtoReg(ID_EXoutMemtoReg),.ID_EXoutMemWrite(ID_EXoutMemWrite),//-->pipe // -->pipe 
    .ID_EXoutALUSrc(ID_EXoutALUSrc),.ID_EXoutRegWrite(),//-->EX // -->pipe &ID
    .ID_EXoutjalsig(ID_EXoutjalsig),//-->EX
    .ID_EXoutIALUCtl(ID_EXoutIALUCtl),//-->EX
    .ID_EXoutlwusig(ID_EXoutlwusig),//-->pipe
    .ID_EXoutSIZE(ID_EXoutSIZE),//-->pipe 
    
    .ID_EXoutjalrsig(ID_EXoutjalrsig),//1bits//-->EX
    .ID_EXoutALUctl(ID_EXoutALUctl),//4bits//-->EX
    .ID_EXoutshamtsig(ID_EXoutshamtsig),//1bits //-->EX
    .ID_EXoutmainshamt(ID_EXoutmainshamt),//5bits//-->EX
    .ID_EXoutToandlinkorder(ID_EXoutToandlinkorder),//32bits//-->pipe
    .ID_EXoutbalandlink(ID_EXoutbalandlink),//1bits//-->EX
    .ID_EXoutfromC(ID_EXoutfromC),.ID_EXoutfromD(ID_EXoutfromD),//32bits //-->EX
    .ID_EXoutImm(ID_EXoutImm),//32bits //-->EX
    .ID_EXouttoEXRs(ID_EXouttoEXRs),//-->EX
    .ID_EXouttoEXRt(ID_inIDEXREGISTERRT),//-->EX & ID!!! 
    .ID_EXouttoEXRd(ID_inIDEXREGISTERRD)//5bits -->EX &ID!!!!

    //output sum 22 
    );


    ex1 userexstage(
    .EX_inIDEXREGISTERRT(ID_inIDEXREGISTERRT),.EX_inIDEXREGISTERRD(ID_inIDEXREGISTERRD),//ID!,ID!
    .EX_inREGDEST(ID_EXoutRegDst),//<--fromp
    //mux5.v //32register sitei
    .EX_inIDEXREGISTERRS(ID_EXouttoEXRs),//&.EX_inIDEXREGISTERRT //<--fromp
    .EX_inEXMEMREGISTERRDRT(ID_inEXMEMREGISTERRDRT),.EX_inMEMWBREGISTERRDRT(ID_inMEMWBRegisterRt),//ID! ID 
    .EX_inEXMEMREGWRITE(ID_inEXMEMREGWRITE),.EX_inMEMWBREGWRITE(ID_inMEMWBREGWRITE),//ID!,ID!
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    .EX_injalsig(ID_EXoutjalsig),.EX_injalrsig(ID_EXoutjalrsig),.EX_inbalal(ID_EXoutbalandlink),//<--fromp
    //allsum.v
    .EX_infromRs(ID_EXoutfromC),.EX_infromRt(ID_EXoutfromD),//<--fromp,<--fromp
    .EX_infromMEMWB(ID_inWBdata),.EX_infromEXMEM(ID_inMEMdata),//ID!,ID!
    //forA(),forB
    .EX_infromshamt(ID_EXoutmainshamt),//<--fromp
    .EX_inshamtsignal(ID_EXoutshamtsig),//<--fromp
    //shamt
    .EX_iniformat(ID_EXoutImm),//<--fromp
    .EX_inALUSRC(ID_EXoutALUSrc),//<--fromp
    //.EX_inalusrcmux 
    .EX_infromALUctl(ID_EXoutALUctl),//<--fromp
    .EX_infromIALUctl(ID_EXoutIALUCtl),//<--fromp

    //----------------------------------------------------------------------------------------------
    .EX_outanswer(EX_outanswer),//32bits//-->pipe
    .EX_outtoANDLINK(EX_outtoANDLINK),//1bits//-->pipe
    .EX_outtopipereg5(EX_outtopipereg5),//5bits//-->pipe
    .EX_outfboutpipe(EX_outfboutpipe)//32bits//-->pipe

    /*signal is all pipeline to pipeline !!!!!!!!!!!!!!!!*/
    );

    exmempipe userpiprexmem(
    .CLOCK(clk),.RESET(rst),
    //
    .EX_MEMinMemRead(ID_inEXMEMMEMREAD),//<--pipe-->ID!
    .EX_MEMinMemtoReg(ID_EXoutMemtoReg),//<--pipe
    .EX_MEMinMemWrite(ID_EXoutMemWrite),//<--pipe
    .EX_MEMinRegWrite(ID_inEXMEMREGWRITE),//<--pipe-->ID!
    .EX_MEMinlwusig(ID_EXoutlwusig),//<--pipe
    .EX_MEMinSIZE(ID_EXoutSIZE),//<--pipe
    /*CTRL 6lines*/
    .EX_MEMinPCadd(ID_EXoutToandlinkorder),//7//<--pipe
    /*PC address*/
    .EX_MEMinALUans(EX_outanswer),//32bits//<--EX
    .EX_MEMinforb(EX_outfboutpipe),//32bits//<--EX
    .EX_MEMinANDLINK(EX_outtoANDLINK),//1bits//10//<--EX
    .EX_MEMinREGISTER(EX_outtopipereg5),//5bits//11//<--EX
    //input 11 
    //------------------------------------------------------>
    
    .EX_MEMoutMemRead(ID_inEXMEMMEMREAD),//-->MEM & ID! 
    .EX_MEMoutMemtoReg(EX_MEMoutMemtoReg),//-->pipe
    .EX_MEMoutMemWrite(EX_MEMoutMemWrite),//-->MEM & output WRITE------------!!!
    .EX_MEMoutRegWrite(ID_inEXMEMREGWRITE),//-->pipe & ID!
    .EX_MEMoutlwusig(EX_MEMoutlwusig),//-->pipe 
    .EX_MEMoutSIZE(EX_MEMoutSIZE),//2bits//-->pipe & output SIZE------------!!!
    /*CTRL 6lines*/
    .EX_MEMoutPCadd(EX_MEMoutPCadd),//7-->pipe
    /*PC address*/
    .EX_MEMoutALUans(ID_inMEMdata),//-->MEM & ID! & output DAD--------------!!!
    .EX_MEMoutforb(EX_MEMoutforb),//--> output DDT --------------------------!!!
    .EX_MEMoutANDLINK(EX_MEMoutANDLINK),//10//-->pipe
    .EX_MEMoutREGISTER(ID_inEXMEMREGISTERRDRT)//11//-->pipe & ID! & EX 

);
    //1
    //pipe --> output & ////EX_MEMoutSIZE is go next pipe!
    assign SIZE = EX_MEMoutSIZE;//2bits
    //2
    //Memwrite ---> MEMstage & WRITE 
    assign WRITE = EX_MEMoutMemWrite; //1bits
    assign DAD = ID_inMEMdata;//ID!
    assign DDT = WRITE ?  EX_MEMoutforb : 32'bz;

    mem1 usermemstage(
    .MEM_inMEMWRITE(EX_MEMoutMemWrite),//-->MEM & WRITE
    .MEM_inMEMREAD(ID_inEXMEMMEMREAD),//-->MEM & ID!!!!
    //------------------------------------------------------------------------------------------
    .MEM_outMREQ(MREQ)//1bits //--->output MREQ!!!
);

    memwbpipe userpipememwb(
    .CLOCK(clk),.RESET(rst),
    //
    .MEM_WBinlastMEMTOREG(EX_MEMoutMemtoReg),//<--pipe
    .MEM_WBinlastREGWRITE(ID_inEXMEMREGWRITE),//<--pipe & ID!
    .MEM_WBinlastSIZE(EX_MEMoutSIZE),//<--pipe--> assign top's output SIZE 
    .MEM_WBinlastLWSIG(EX_MEMoutlwusig),//<--pipe 
    /*CTRL 4lines */
    .MEM_WBinlastlwans(DDT),//5//<-------------------------------------DDT!!!!!!!!!!!!!!!!!!!!!!
    .MEM_WBinlastPC(EX_MEMoutPCadd),//32bits//<--pipe
    .MEM_WBinlastRform(ID_inMEMdata),//32bits//<-- pipe & ID! 
    .MEM_WBinlastandlinlsig(EX_MEMoutANDLINK),//1bits//<-------pipe
    .MEM_WBinlastwherereg(ID_inEXMEMREGISTERRDRT),//5bits<--pipe & ID!!!
    //input 9lines + CLOCK RESET

    //--------------------------------------------------------------------------------------------

    .MEM_WBoutMEMTOREG(MEM_WBoutMEMTOREG),//-->WB
    .MEM_WBoutREGWRITE(ID_inMEMWBREGWRITE),//-->WB & ID!
    .MEM_WBoutSIZE(MEM_WBoutSIZE),//2bits//-->WB
    .MEM_WBoutLWSIG(MEM_WBoutLWSIG),//1bits//-->WB
    /*CTRL 4lines */
    .MEM_WBoutlwans(MEM_WBoutlwans),//32bits//-->WB
    .MEM_WBoutPC(MEM_WBoutPC),//32bits//-->WB
    .MEM_WBoutRform(MEM_WBoutRform),//32bits//-->WB
    .MEM_WBoutandlinlsig(MEM_WBoutandlinlsig),//-->WB
    .MEM_WBoutwherereg(ID_inMEMWBRegisterRt)//-->ID
    //output 9lines 
    );


    we1 userwbstage(
    .WB_infromplw(MEM_WBoutlwans),//32bits//<--pipe
    .WB_inLASTSIZE(MEM_WBoutSIZE),//2bits//<--pipe
    .WB_insignLW(MEM_WBoutLWSIG),//1bits//<--pipe
    //lwunsigned.v // out is wire "EDITLOAD"
    .WB_infrompaddANS(MEM_WBoutRform),//32bits//<--pipe(Rform...)
    .WB_infrompMEMTOREG(MEM_WBoutMEMTOREG),//1bits//<--pipe
    //mux.v(lwRmux) --> out:wire LASTJUDGE
    .WB_inALINKPC(MEM_WBoutPC),//32bits//<--pipe
    .WB_inLINKSIG(MEM_WBoutandlinlsig),//1bits//<--pipe
    //---------------------------------------------------------------------------------------------------------------
    .WB_outGOREGDATA(ID_inDATAIN32)
    );


endmodule

