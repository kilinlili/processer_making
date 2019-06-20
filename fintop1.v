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
    wire [4:0]ID_inWRITEADD;
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
    //in 0

    wire ID_EXoutRegDst,ID_EXoutMemtoReg,ID_EXoutMemWrite,ID_EXoutALUSrc,ID_EXoutRegWrite;
    wire ID_EXoutjalsig,ID_EXoutlwusig;
    wire [3:0] ID_EXoutIALUCtl;
    wire [1:0] ID_EXoutSIZE;
    //out1
    wire ID_EXoutjalrsig;//1bits
    wire [3:0] ID_EXoutALUctl;//4bits
    wire ID_EXoutshamtsig;//1bits 
    wire [4:0] ID_EXoutmainshamt;//5bits
    wire [31:0] ID_EXoutToandlinkorder;//32bits
    wire ID_EXoutbalandlink;//1bits
    wire [31:0] ID_EXoutfromC,ID_EXoutfromD,ID_EXoutImm;//32bits 
    wire [4:0] ID_EXouttoEXRs;//5bits 
    //out2 

//-----------------------------------------------------------------------------------
//EXSTAGE




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
    .ID_inWRITEADD(ID_inWRITEADD),//5bits <--register locate?//check1
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

    .ID_EXoutRegDst(ID_EXoutRegDst),.ID_EXoutMemRead(ID_inIDEXMEMREAD),//-->EX ///-->pipe & ID!!!!
    .ID_EXoutMemtoReg(ID_EXoutMemtoReg),.ID_EXoutMemWrite(ID_EXoutMemWrite),//-->pipe // -->pipe
    .ID_EXoutALUSrc(ID_EXoutALUSrc),.ID_EXoutRegWrite(ID_EXoutRegWrite),//-->EX // -->pipe
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
    .EX_inIDEXREGISTERRT(),.EX_inIDEXREGISTERRD(),
    .EX_inREGDEST(),
    //mux5.v //32register sitei
    .EX_inIDEXREGISTERRS(),//&.EX_inIDEXREGISTERRT
    .EX_inEXMEMREGISTERRDRT(),.EX_inMEMWBREGISTERRDRT(),
    .EX_inEXMEMREGWRITE(),.EX_inMEMWBREGWRITE(),
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    .EX_injalsig(),.EX_injalrsig(),.EX_inbalal(),
    //allsum.v
    .EX_infromRs(),.EX_infromRt(),.EX_infromMEMWB(),.EX_infromEXMEM(),
    //forA(),forB
    .EX_infromshamt(),
    .EX_inshamtsignal(),
    //shamt
    .EX_iniformat(),
    .EX_inALUSRC(),
    //.EX_inalusrcmux 
    .EX_infromALUctl(),
    .EX_infromIALUctl(),

    //----------------------------------------------------------------------------------------------
    .EX_outanswer(),
    .EX_outtoANDLINK(),//-------------->to pipeline
    .EX_outtopipereg5(),
    .EX_outfboutpipe()
    //ALU
    );

    exmempipe userpiprexmem(
    .CLOCK(),.RESET(),
    //
    .EX_MEMinMemRead(),.EX_MEMinMemtoReg(),.EX_MEMinMemWrite(),.EX_MEMinRegWrite(),
    .EX_MEMinlwusig(),
    .EX_MEMinSIZE(),
    /*CTRL 6lines*/
    .EX_MEMinPCadd(),//7
    /*PC address*/
    .EX_MEMinALUans(),.EX_MEMinforb(),//8(),9
    .EX_MEMinANDLINK(),//10
    .EX_MEMinREGISTER(),//11

    //------------------------------------------------------>
    
    .EX_MEMoutMemRead(),.EX_MEMoutMemtoReg(),.EX_MEMoutMemWrite(),.EX_MEMoutRegWrite(),
    .EX_MEMoutlwusig(),
    .EX_MEMoutSIZE(),
    /*CTRL 6lines*/
    .EX_MEMoutPCadd(),//7
    /*PC address*/
    .EX_MEMoutALUans(),.EX_MEMoutforb(),//8(),9
    .EX_MEMoutANDLINK(),//10
    .EX_MEMoutREGISTER()//11

);

    mem1 usermemstage(
    .MEM_inMEMWRITE(),.MEM_inMEMREAD(),
    //------------------------------------------------------------------------------------------
    .MEM_outMREQ()
);


    memwbpipe userpipememwb(
    .CLOCK(),.RESET(),
    //
    .MEM_WBinlastMEMTOREG(),.MEM_WBinlastREGWRITE(),
    .MEM_WBinlastSIZE(),
    .MEM_WBinlastLWSIG(),
    /*CTRL 4lines */
    .MEM_WBinlastlwans(),//5
    .MEM_WBinlastPC(),//6
    .MEM_WBinlastRform(),//7
    .MEM_WBinlastandlinlsig(),//8
    .MEM_WBinlastwherereg(),//9
    //input 9lines + CLOCK RESET

    //---------------------------------------->>>
    .MEM_WBoutMEMTOREG(),.MEM_WBoutREGWRITE(),
    .MEM_WBoutSIZE(),
    .MEM_WBoutLWSIG(),
    /*CTRL 4lines */
    .MEM_WBoutlwans(),//5
    .MEM_WBoutPC(),//6
    .MEM_WBoutRform(),//7
    .MEM_WBoutandlinlsig(),//8
    .MEM_WBoutwherereg()//9
    //output 9lines 
    );


    we1 userwbstage(
    .WB_infromplw(),
    .WB_inLASTSIZE(),
    .WB_insignLW(),
    //lwunsigned.v // out is wire "EDITLOAD"
    .WB_infrompaddANS(),
    .WB_infrompMEMTOREG(),
    //mux.v(lwRmux) --> out:wire LASTJUDGE
    .WB_inALINKPC(),
    .WB_inLINKSIG(),
    //---------------------------------------------------------------------------------------------------------------
    .WB_outGOREGDATA()
    );


endmodule

