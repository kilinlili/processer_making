module id1(
    CLOCK,RESET,
    IFID_ORDER,
    IFID_PCADD4,
    //4
    ID_inMEMWBREGWRITE,//ID_inMEMID_inWBREGWRITE
    ID_inMEMWBRegisterRt,//----------------------------
    ID_inWBdata,ID_inMEMdata,
    ID_inIDEXMEMREAD,ID_inIDEXREGWRITE,
    ID_inIDEXREGISTERRT,ID_inIDEXREGISTERRD,
    ID_inEXMEMREGWRITE,ID_inEXMEMMEMREAD,
    ID_inEXMEMREGISTERRDRT,
    ID_inDATAIN32,
    //13
    //input sum is 17

    ID_outIFIDWRITE,ID_outPCWRITE,
    ID_outtoandlinkorder,
    ID_outjumpaddress,
    //4
    ID_outtoEXRd,
    ID_outtoEXRs,
    ID_outtoEXRt,
    ID_outImmout,
    ID_outtoshamt,///
    ID_outDATA1,ID_outDATA2,
    ID_outbaddress,
    ID_outbalandlink,
    ID_outbeqtojrjalr32,
    //10
    ID_outtoIFpcsrc,
    ID_outtoIFjump,
    ID_outALUctltopipe,
    ID_outALUopshamtsig,///
    ID_outALUopjalrsig,
    ID_outALUoprjump,
    //6
    ID_outRegDst,ID_outMemRead,ID_outMemtoReg,ID_outMemWrite,ID_outALUSrc,ID_outRegWrite,
    ID_outjalsig,
    ID_outIALUCtl,
    ID_outlwusig,
    ID_outSIZE,
    //10

    //output is 30
);

/////////////////////////////////////////////////////////////////////////////////////////////////

    input CLOCK,RESET;
    input  [31:0] IFID_ORDER;//from IFIDpipeline
    input  [31:0] IFID_PCADD4;//from IFIDpipeline
    //4
    input ID_inMEMWBREGWRITE;
    input [4:0] ID_inMEMWBRegisterRt;
    input [31:0] ID_inWBdata,ID_inMEMdata;//& wire IFIDRs & wire IFIDRt
    input [31:0] ID_inDATAIN32;
    input ID_inIDEXMEMREAD,ID_inIDEXREGWRITE;
    input [4:0] ID_inIDEXREGISTERRT,ID_inIDEXREGISTERRD;
    input ID_inEXMEMREGWRITE,ID_inEXMEMMEMREAD;
    input [4:0] ID_inEXMEMREGISTERRDRT;
    //13

    //
    output ID_outIFIDWRITE,ID_outPCWRITE;
    output [31:0] ID_outtoandlinkorder;
    output [31:0] ID_outjumpaddress;
    output [31:0] ID_outDATA1,ID_outDATA2;
    output [4:0] ID_outtoEXRd;
    output [4:0] ID_outtoEXRs;
    output [4:0] ID_outtoEXRt;
    output [4:0] ID_outtoshamt;
    //10

    output ID_outRegDst,ID_outMemRead,ID_outMemtoReg,ID_outMemWrite,ID_outALUSrc,ID_outRegWrite;
    output ID_outjalsig;
    output [3:0] ID_outIALUCtl;
    output ID_outlwusig;
    output [1:0] ID_outSIZE;
    //10

    output [31:0] ID_outbaddress;
    output ID_outbalandlink;
    output [31:0] ID_outbeqtojrjalr32;
    output ID_outtoIFpcsrc;
    output [1:0]ID_outtoIFjump;//from ctrlmux to IFStage's mux data //assign ID_outtoIFjump = (wire)toJump
    output [3:0]ID_outALUctltopipe;
    output ID_outALUopshamtsig;
    output ID_outALUopjalrsig;
    output [1:0] ID_outALUoprjump;
    output [31:0] ID_outImmout;
    //10
    //output sum is 30
    
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    wire [5:0]IFIDop = IFID_ORDER[31:26];//Q1
    wire [4:0]IFIDRd = IFID_ORDER[15:11];//-->to for2 & to pipeline(output ID_outtoEXRd) 
    wire [4:0]IFIDRs = IFID_ORDER[25:21];//-->to for2 & to pipeline(output ID_outtoEXRs) & register
    wire [4:0]IFIDRt = IFID_ORDER[20:16];//-->to for2 & to pipeline(output ID_outtoEXRt) & register 
    wire [4:0]IFIDsh = IFID_ORDER[10: 6];//-->to pipeline --> EXstage shamtmux --> ALU
    wire [25:0]IFIDtwentysix = IFID_ORDER[25:0];//-->to 2bitleft to jump//out is wire "nohead4"
    //rf32 is registerfile
    wire [1:0]forc,ford;
    //forwarding unit2 
    wire [15:0] Imm = IFID_ORDER[15:0];
    wire [31:0]expandout;//------------------------------->to 2bitleft & to ALUCtl & to pipeline //assign funccode[5:0] = expandout[5:0]?????
    //16_32.v
    wire [31:0] fromRegRs,fromRegRt;//from register to forward C & D
    wire [31:0] Ctobeqandpipe,Dtobeqandpipe;//forwardC out & forwardD out
    wire [31:0] tobranchaddA;//&input branchadder B 
    wire [5:0] aluopandbeqjumpfunccode;//to aluop & to branch jumpctl
    wire [3:0] beqjumpcode;//from beqjumpctl to main beq
    wire tobeqand;//from main_beq to iand

    /*CTRL wires to  ctrlmux*/
    wire CTRLRegDst,CTRLBranch,CTRLMemRead,CTRLMemtoReg,CTRLMemWrite,CTRLALUSrc,CTRLRegWrite;
    wire CTRLjalsig;
    wire [3:0] CTRLIALUCtl;
    wire [1:0] CTRLALUOp,CTRLJump;
    wire CTRLlwusig;
    wire [1:0] CTRLSIZE;
    /*CTRL wires*/

    /*CTRLmux out to pipeline*/
    wire toRegDst,toBranch,toMemRead,toMemtoReg,toMemWrite,toALUSrc,toRegWrite;
    wire tojalsig;
    wire [3:0] toIALUCtl;
    wire [1:0] toALUOp,toJump;//toJump is output //toALUOp & toJump & toBranch is not pipeline 
    wire tolwusig;
    wire [1:0] toSIZE;
    // next is pipeline ,so  assign ID_out~ (sum 10line)
    wire zerosignal;//---------------------------------------------------->from hazard to ctrlmux 
    /*CTRLmux out to pipeline*/
    wire [27:0] nohead4;
    //2bit_leftjump.v
    wire [3:0] head4 = IFID_PCADD4[31:28];//Q1
    //from IFIDpipeline,PC+4[31:28] to jhead32.v//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Q2
    assign ID_outtoEXRd = IFID_ORDER[15:11];//----->to pipeline 
    assign ID_outtoEXRs = IFID_ORDER[25:21];//----->to pipeline
    assign ID_outtoEXRt = IFID_ORDER[20:16];//----->to pipeline //2 line? --> 1line change OK
    assign ID_outtoshamt= IFID_ORDER[10:6]; //----->to pipeline  // or IFIDsh OK
    assign ID_outtoandlinkorder = IFID_PCADD4;//----------------------------->to pipeline

    forwarding2 forward2(
        .ForwardC(forc),.ForwardD(ford),//to forwardc & fowardd
        .EX_MEM_RegWrite(ID_inEXMEMREGWRITE),.EX_MEM_Memread(ID_inEXMEMMEMREAD),
        .MEM_WB_RegWrite(ID_inMEMWBREGWRITE),
        .EX_MEM_RegisterR(ID_inEXMEMREGISTERRDRT),.MEM_WB_RegisterRt(ID_inMEMWBRegisterRt),
        .IF_ID_RegisterRs(IFIDRs),.IF_ID_RegisterRt(IFIDRt)
    );
    expand expand1632(
        .datain(Imm),.dataout(expandout)//----------to pipeline & to 2bit left & (ALUOp & beq_jumpctl)under 6bits  
    );
    assign ID_outImmout = expandout;//------------------------------>to pipeline
    assign aluopandbeqjumpfunccode = expandout[5:0];//----------------------->to ALUop & to beq_jumpctl

    //////////////////////////////////////
    fmux forwardc(
        .data1(fromRegRs),.data2(ID_inMEMdata),.data3(ID_inWBdata),//00,10MEM,01WB
        .signal(forc),
        .out(Ctobeqandpipe)//to mainbeq & to pipeline
    );
    assign ID_outDATA1 = Ctobeqandpipe;//============================>to pipeline 
    fmux forwardd(
        .data1(fromRegRt),.data2(ID_inMEMdata),.data3(ID_inWBdata),//00,10MEM,01WB
        .signal(ford),
        .out(Dtobeqandpipe)//to mainbeq & to pipeline 
    );
    assign ID_outDATA2 = Dtobeqandpipe;//============================>to pipeline

    twobitl twoleft(
        .A(expandout),.B(tobranchaddA)
    );

    adder branchadd(
        .data1(tobranchaddA),.data2(IFID_PCADD4),//data1 is from 2bitleft ,data2 is from IFIDPIPELINE
        .adder_out(ID_outbaddress)//-->IF!
    );

    beqjump beqjumpctrl(
        .Rt(IFIDRt),.OP(IFIDop),.FuncCode(aluopandbeqjumpfunccode)/*6bit*/,
        .out(beqjumpcode)/*4bit*/
    );

    mainbeq i_mainbeq(
        .fromreg1(Ctobeqandpipe)/*32*/,.fromreg2(Dtobeqandpipe)/*32bit*/,.ctlbeq(beqjumpcode)/*6bit in*/,
        .branchin(tobeqand),.alout(ID_outbalandlink),//wire:tobeqand -> iand(R)//output ID_outbalandlink
        .jrre(ID_outbeqtojrjalr32)//-->IF!
    );
    
    
    mainCTL name_mainCTRL(
        .OP(IFIDop),//input IFIDop[5:0] =IFID_ORDER[31:26]
        //all ctrlmux
        .RegDst(CTRLRegDst),.Branch(CTRLBranch),.MemRead(CTRLMemRead),.MemtoReg(CTRLMemtoReg),.MemWrite(CTRLMemWrite),.ALUSrc(CTRLALUSrc),.RegWrite(CTRLRegWrite),//allwire
        .jalsig(CTRLjalsig),
        .IALUCtl(CTRLIALUCtl),
        .ALUOp(CTRLALUOp),
        .Jump(CTRLJump),
        .lwusig(CTRLlwusig),.SIZE(CTRLSIZE)
    );

    ctrlmux i_ctlmux(
        .inRegDst(CTRLRegDst),.inBranch(CTRLBranch),.inMemRead(CTRLMemRead),.inMemtoReg(CTRLMemtoReg),.inMemWrite(CTRLMemWrite),.inALUSrc(CTRLALUSrc),.inRegWrite(CTRLRegWrite),
        .injalsig(CTRLjalsig),
        .inIALUCtl(CTRLIALUCtl),
        .inALUOp(CTRLALUOp),
        .inJump(CTRLJump),
        .inlwusig(CTRLlwusig),.inSIZE(CTRLSIZE),
        //input
        .zerosig(zerosignal),// ===================================> input wire zerosignal
        //output 
        .outRegDst(toRegDst),.outBranch(toBranch)/*iand's L*/,.outMemRead(toMemRead),.outMemtoReg(toMemtoReg),.outMemWrite(toMemWrite),.outALUSrc(toALUSrc),.outRegWrite(toRegWrite),
        .outjalsig(tojalsig),
        .outIALUCtl(toIALUCtl),
        .outALUOp(toALUOp),
        .outJump(toJump),
        .outlwusig(tolwusig),.outSIZE(toSIZE)
    );
    
    assign ID_outtoIFjump = toJump;//from ctrlmux to IFstage's jmlt.v 's jump
    //----------------------------------------------------------------------------------------------------------->to pipeline
    assign ID_outRegDst  = toRegDst;
    assign ID_outMemRead = toMemRead;
    assign ID_outMemtoReg= toMemtoReg;
    assign ID_outMemWrite= toMemWrite;
    assign ID_outALUSrc  = toALUSrc;
    assign ID_outRegWrite= toRegWrite;
    assign ID_outjalsig  = tojalsig;
    assign ID_outIALUCtl = toIALUCtl;
    assign ID_outlwusig  = tolwusig;
    assign ID_outSIZE    = toSIZE;
    //sum : 10 line ====> OK
    //----------------------------------------------------------------------------------------------------------->to pipeline

    ALUControl iALUctl(
        .ALUOp(toALUOp),.FuncCode(aluopandbeqjumpfunccode),//wire input
        .ALUCtl(ID_outALUctltopipe),.jalrsig(ID_outALUopjalrsig),.shamtsig(ID_outALUopshamtsig),//-------------------------------------------------output; to pipeline 
        .rjump(ID_outALUoprjump)//----------------------------------------------------------------------------------------output ->IFstage
    );
    iand andtoIFstage(
        .left(toBranch),.right(tobeqand),
        .ans(ID_outtoIFpcsrc)//--> IF!
    );
    hazard mainhazard(
        .muxzero(zerosignal),//wire out //to crtlmux
        .pcwrite(ID_outPCWRITE),//output //to userpc
        .IF_IDwrite(ID_outIFIDWRITE),//output //to ifidregister 

        .ID_EX_MemRead(ID_inIDEXMEMREAD),.ID_EX_Regwrite(ID_inIDEXREGWRITE),//input 
        .ID_EX_RegisterRt(ID_inIDEXREGISTERRT),//input
        .ID_EX_RegisterRd(ID_inIDEXREGISTERRD),//input

        .EX_MEM_Regwrite(ID_inEXMEMREGWRITE),.EX_MEM_Memread(ID_inEXMEMMEMREAD),//input
        .EX_MEM_RegisterRt(ID_inEXMEMREGISTERRDRT),//input

        .IF_ID_RegisterRs(IFIDRs),.IF_ID_RegisterRt(IFIDRt),//wire in 
        .branch(toBranch)//wire in
    );
    //2bit_left_jump
    twobitljump tojumpshift(
        .A(IFIDtwentysix),.B(nohead4)//A:in,B:out
    );

    jhead tojump32bits(
        .frompc(head4),.twei(nohead4),// in 4bit head4 ,28bit nohead
        .out(ID_outjumpaddress)//--------------------------------------------------------->to IFstage!
    );

    //----------------------------------------------------------------------------------------
    rf32x32 mainREG(
		.clk(CLOCK),.reset(RESET),//input
		// Inputs
		.wr_n(ID_inMEMWBREGWRITE),//input regwrite
		.rd1_addr(IFIDRs), .rd2_addr(IFIDRt), //wire in 5bits
        .wr_addr(ID_inMEMWBRegisterRt),//input "out of IDstage" 5bits
		.data_in(ID_inDATAIN32),//input "out of IDstage" 32bits 
		.data1_out(fromRegRs), .data2_out(fromRegRt)//wire out 32bits //to forC & forD
    );

endmodule

