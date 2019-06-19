module id1(
    input  [31:0] thirtytwo,//from IFIDpipeline
    input [31:0] frompcadd4,//from IFIDpipeline
    output [31:0] toandlinkorder,//assign this = frompcadd4 ---->to pipeline
    output [31:0] jumpaddress,//in wire 28bit & 4bit = 32bits ->output IFstage

    output [4:0] toEXRd,
    output [4:0] toEXRs,
    output [4:0] toEXRt,
    output [4:0] toshamt,
    /*32bits main data*/

    //input EXMEMREGWRITE-OK //input EXMEMMEMREAD =>OK
    input MEMWBRegWrite,
    input [4:0] MEMWBRegisterRt,//input EXMEMREGISTERRDRT->OK
    //wire in [4:0]IFIDRegisterRs,IFIDRegisterRt
    /*forwarding_unit2*/

    output [31:0] Immout,
    //16_32.v

    input [31:0] WBdata,MEMdata,//& fromRs(32),fromRt(32)
    output [31:0] fromC,fromD,
    //forwardC & forwardD


    //from 2bitleft & from IFID pipeline(this!)<-input frompcadd4
    output [31:0] branchaddanswer,//from branchadd IFstage's mux data;
    //adder.v(branchadd)

    output balandlink,//from main_beq to pipeline
    output [31:0] beqtojrjalr32,//from main_beq to jrreg data
    //main_beq.v 
    //beq_jumpCTL.v is all wire 

    output toIFpcsrc,//from iand to IFstage's mux sig
    output [1:0]toIFjump,//from ctrlmux to IFStage's mux data //assign toIFjump = (wire)toJump

    output [3:0]ALUctltopipe,
    output ALUopshamtsig,
    output ALUopjalrsig,
    output /*wire ??*/[1:0] ALUoprjump,
    /*
    input:wire fromCTRL's ALUop[1:0]
    *///ALUCTL.v
    //CTRL.v ->ctrlmux.v is  almost wire...
    /*ctrl output to IDEXPIPELINE!!!!!!!!!!!!!!!*/
    output goRegDst,goMemRead,goMemtoReg,goMemWrite,goALUSrc,goRegWrite,
    output gojalsig,
    output [3:0] goIALUCtl,
    output golwusig,
    output [1:0] goSIZE,
    //ALL assign wo TUKAU!
    /*ctrl output to IDEXPIPELINE!!!!!!!!!!!!!!!*/

    /*harzard*/
    input IDEXMEMREAD,IDEXREGWRITE,
    input [4:0] IDEXREGISTERRT,IDEXREGISTERRD,
    input EXMEMREGWRITE,EXMEMMEMREAD,
    input [4:0] EXMEMREGISTERRDRT,
    output IFIDWRITE,PCWRITE,// & wire out ctrlmux 
    /*harzard*/

//output:30 OK ! my figure is same too!
//input:16 OK ! my figure is same too! 

    /*register*/
    input CLOCK,RESET,
    input [4:0] WRITEADD,//& wire IFIDRs & wire IFIDRt
    input [31:0] DATAIN32,
    input WBREGWRITE
    //wire in "Rs, Rt,"2line
    //wire out"fromRs,fromRt"

    /*register*/

    //out:wire fromRegRs,fromRegRt


);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    wire [5:0]IFIDop = thirtytwo[31:26];//Q1
    wire [4:0]IFIDRd = thirtytwo[15:11];//-->to for2 & to pipeline(output toEXRd) 
    wire [4:0]IFIDRs = thirtytwo[25:21];//-->to for2 & to pipeline(output toEXRs) & register
    wire [4:0]IFIDRt = thirtytwo[20:16];//-->to for2 & to pipeline(output toEXRt) & register 
    wire [4:0]IFIDsh = thirtytwo[10: 6];//-->to pipeline --> EXstage shamtmux --> ALU
    wire [25:0]IFIDtwentysix = thirtytwo[25:0];//-->to 2bitleft to jump//out is wire "nohead4"
    //rf32 is registerfile
    wire [1:0]forc,ford;
    //forwarding unit2 
    wire [15:0] Imm = thirtytwo[15:0];
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
    // next is pipeline ,so  assign go~ (sum 10line)
    wire zerosignal;//---------------------------------------------------->from hazard to ctrlmux 
    /*CTRLmux out to pipeline*/
    wire [27:0] nohead4;
    //2bit_leftjump.v
    wire [3:0] head4 = frompcadd4[31:28];//Q1
    //from IFIDpipeline,PC+4[31:28] to jhead32.v//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Q2
    assign toEXRd = thirtytwo[15:11];//----->to pipeline 
    assign toEXRs = thirtytwo[25:21];//----->to pipeline
    assign toEXRt = thirtytwo[20:16];//----->to pipeline //2 line? --> 1line change OK
    assign toshamt= thirtytwo[10:6]; //----->to pipeline  // or IFIDsh OK
    assign toandlinkorder = frompcadd4;//----------------------------->to pipeline

    forwarding2 forward2(
        .ForwardC(forc),.ForwardD(ford),//to forwardc & fowardd
        .EX_MEM_RegWrite(EXMEMREGWRITE),.EX_MEM_Memread(EXMEMMEMREAD),
        .MEM_WB_RegWrite(MEMWBRegWrite),
        .EX_MEM_RegisterR(EXMEMREGISTERRDRT),.MEM_WB_RegisterRt(MEMWBRegisterRt),
        .IF_ID_RegisterRs(IFIDRs),.IF_ID_RegisterRt(IFIDRt)
    );
    expand expand1632(
        .datain(Imm),.dataout(expandout)//----------to pipeline & to 2bit left & (ALUOp & beq_jumpctl)under 6bits  
    );
    assign Immout = expandout;//------------------------------>to pipeline
    assign aluopandbeqjumpfunccode = expandout[5:0];//----------------------->to ALUop & to beq_jumpctl

    //////////////////////////////////////
    fmux forwardc(
        .data1(fromRegRs),.data2(MEMdata),.data3(WBdata),//00,10MEM,01WB
        .signal(forc),
        .out(Ctobeqandpipe)//to mainbeq & to pipeline
    );
    assign fromC = Ctobeqandpipe;//============================>to pipeline 
    fmux forwardd(
        .data1(fromRegRt),.data2(MEMdata),.data3(WBdata),//00,10MEM,01WB
        .signal(ford),
        .out(Dtobeqandpipe)//to mainbeq & to pipeline 
    );
    assign fromD = Dtobeqandpipe;//============================>to pipeline

    /////////////////////////////////////////////////
    //register file 
    /*
    rx32x32 mainregister(
        //clk,reset???????
        .wr_n(),//????????????????
        .rd1_addr(IFIDRs),.re2_addr(IFIDRt),.wr_addr(writeregi),.data_in(writeregidata),
        .data1_out(fromRegRs),.data2_out(fromRegRt)
    );
    */
    /////////////////////////////////////////////////

    twobitl twoleft(
        .A(expandout),.B(tobranchaddA)
    );

    adder branchadd(
        .data1(tobranchaddA),.data2(frompcadd4),//data1 is from 2bitleft ,data2 is from IFIDPIPELINE
        .adder_out(branchaddanswer)//
    );

    beqjump beqjumpctrl(
        .Rt(IFIDRt),.OP(IFIDop),.FuncCode(aluopandbeqjumpfunccode)/*6bit*/,
        .out(beqjumpcode)/*4bit*/
    );

    mainbeq i_mainbeq(
        .fromreg1(Ctobeqandpipe)/*32*/,.fromreg2(Dtobeqandpipe)/*32bit*/,.ctlbeq(beqjumpcode)/*6bit in*/,
        .branchin(tobeqand),.alout(balandlink),//wire:tobeqand -> iand(R)//output balandlink
        .jrre(beqtojrjalr32)//32bit jump saki data
    );
    
    
    mainCTL name_mainCTRL(
        .OP(IFIDop),//input IFIDop[5:0] =thirtytwo[31:26]
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
    
    assign toIFjump = toJump;//from ctrlmux to IFstage's jmlt.v 's jump
    //----------------------------------------------------------------------------------------------------------->to pipeline
    assign goRegDst  = toRegDst;
    assign goMemRead = toMemRead;
    assign goMemtoReg= toMemtoReg;
    assign goMemWrite= toMemWrite;
    assign goALUSrc  = toALUSrc;
    assign goRegWrite= toRegWrite;
    assign gojalsig  = tojalsig;
    assign goIALUCtl = toIALUCtl;
    assign golwusig  = tolwusig;
    assign goSIZE    = toSIZE;
    //sum : 10 line ====> OK
    //----------------------------------------------------------------------------------------------------------->to pipeline

    ALUControl iALUctl(
        .ALUOp(toALUOp),.FuncCode(aluopandbeqjumpfunccode),//wire input
        .ALUCtl(ALUctltopipe),.jalrsig(ALUopjalrsig),.shamtsig(ALUopshamtsig),//-------------------------------------------------output; to pipeline 
        .rjump(ALUoprjump)//----------------------------------------------------------------------------------------output ->IFstage
    );
    iand andtoIFstage(
        .left(toBranch),.right(tobeqand),
        .ans(toIFpcsrc)//output OK 
    );
    hazard mainhazard(
        .muxzero(zerosignal),//wire out //to crtlmux
        .pcwrite(PCWRITE),//output //to userpc
        .IF_IDwrite(IFIDWRITE),//output //to ifidregister 

        .ID_EX_MemRead(IDEXMEMREAD),.ID_EX_Regwrite(IDEXREGWRITE),//input 
        .ID_EX_RegisterRt(IDEXREGISTERRT),//input
        .ID_EX_RegisterRd(IDEXREGISTERRD),//input

        .EX_MEM_Regwrite(EXMEMREGWRITE),.EX_MEM_Memread(EXMEMMEMREAD),//input
        .EX_MEM_RegisterRt(EXMEMREGISTERRDRT),//input

        .IF_ID_RegisterRs(IFIDRs),.IF_ID_RegisterRt(IFIDRt),//wire in 
        .branch(toBranch)//wire in
    );
    //2bit_left_jump
    twobitljump tojumpshift(
        .A(IFIDtwentysix),.B(nohead4)//A:in,B:out
    );

    jhead tojump32bits(
        .frompc(head4),.twei(nohead4),// in 4bit head4 ,28bit nohead
        .out(jumpaddress)//--------------------------------------------------------->to IFstage!
    );

    //----------------------------------------------------------------------------------------
    rf32x32 mainREG(
		.clk(CLOCK),.reset(RESET),//input
		// Inputs
		.wr_n(WBREGWRITE),//input regwrite
		.rd1_addr(IFIDRs), .rd2_addr(IFIDRt), //wire in 5bits
        .wr_addr(WRITEADD),//input "out of IDstage" 5bits
		.data_in(DATAIN32),//input "out of IDstage" 32bits 
		.data1_out(fromRegRs), .data2_out(fromRegRt)//wire out 32bits //to forC & forD
    );




endmodule









///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module forwarding2(ForwardC,ForwardD,
EX_MEM_RegWrite,EX_MEM_Memread,//input 
MEM_WB_RegWrite,//input
EX_MEM_RegisterR,MEM_WB_RegisterRt,//input
IF_ID_RegisterRs,IF_ID_RegisterRt//wire in
);
    //in:6,out:2
    //beq for

    input  EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite;
    input [4:0] EX_MEM_RegisterR,MEM_WB_RegisterRt/*lw*/;
    input [4:0] IF_ID_RegisterRs,IF_ID_RegisterRt; 

    output [1:0] ForwardC,ForwardD;

    assign ForwardC=forward(EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite, EX_MEM_RegisterR/*RdRt*/,MEM_WB_RegisterRt,IF_ID_RegisterRs);
    assign ForwardD=forward(EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite, EX_MEM_RegisterR/*RdRt*/,MEM_WB_RegisterRt,IF_ID_RegisterRt);

    function [1:0] forward;
        input  EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite;
        input [4:0] EX_MEM_RegisterR,MEM_WB_RegisterRt/*lw*/;
        input [4:0] IF_ID_RegisterR;

        if(EX_MEM_RegWrite & (EX_MEM_Memread==0) & ((EX_MEM_RegisterR == IF_ID_RegisterR))) 
            //add-1st-beq OK
            //addi-1st-beq OK
            forward = 2'b10;
        else if(MEM_WB_RegWrite & EX_MEM_Memread & (MEM_WB_RegisterRt == IF_ID_RegisterR) & 
        ~(EX_MEM_RegWrite & (EX_MEM_Memread==0) & ((EX_MEM_RegisterR == IF_ID_RegisterR))))
            //lw-2st-beq 
            forward = 2'b01;
        else
            forward = 2'b00;
    endfunction
endmodule

//16 ->>>>>>>32 bit expand
module expand(
        input[15:0]datain,
        output[31:0]dataout
);
        function [31:0] extend;
        input[15:0] A;
        if(A[15]==1'b0) extend = 32'h0000 + A;
        else            extend = 32'hffff0000 + A;
        endfunction
        assign dataout= extend(datain);
endmodule
////////////////////////////////////0 ume sitekureru?????

//forwarding mux
module fmux(
    input[31:0]data1,data2,data3,
    input [1:0] signal,
    output [31:0] out
);
    function [31:0]fbmux;
        input[31:0]data1,data2,data3;
        input [1:0] signal;
        if(signal[1] == 1'b1) //EXMEM //11,10
            fbmux = data2;
        else if(signal == 2'b00) //IDEX 
            fbmux = data1; 
        else if(signal == 2'b01) //MEMWB
            fbmux  = data3;
 
    endfunction // hoge
    assign out = fbmux(data1,data2,data3,signal);
endmodule

/*register file ????*/
/*register file*/

//2bitleft
//2bit left shift
module twobitl(
        input[31:0] A,
        output[31:0] B
);
        assign B = (A<<2);
endmodule

//branchadder
module adder(data1,data2,adder_out);
    input [31:0] data1,data2;
    output[31:0] adder_out;
    assign adder_out = data1+data2;
endmodule

//mainbeq
module mainbeq(fromreg1,fromreg2,ctlbeq,branchin,alout,jrre);
    //in 3 (reg x2) out 1 or 2
    input [31:0] fromreg1,fromreg2;
    input [3:0] ctlbeq;//<-out

    output reg branchin;//to and culicurate
    output reg alout;
    output reg [31:0] jrre;

    always @(ctlbeq,fromreg1,fromreg2) begin
        if(ctlbeq == 4'b0000) begin //beq
            branchin <= fromreg1 == fromreg2 ? 1:0;
            alout <= 1'b0;
            jrre <=32'h00000000;//nanndemo OK
        end
        else if(ctlbeq == 4'b0001) begin //bne 
            branchin <= fromreg1 != fromreg2 ? 1:0;
            alout <= 1'b0;
            jrre <=32'h00000000;//nanndemo OK
        end 
        else if(ctlbeq == 4'b0010) begin //blez 
            branchin = fromreg1 <=0 ? 1:0;
            alout <= 1'b0;
            jrre <=32'h00000000;//nanndemo OK
        end
        else if(ctlbeq == 4'b0011) begin //bgtz
            branchin <= fromreg1 > 0? 1:0;
            alout <= 1'b0;
            jrre <=32'h00000000;//nanndemo OK
        end
        else if(ctlbeq == 4'b0100) begin //bal//bgez
            branchin <= fromreg1 <0 ? 1:0;
            alout <= 1'b0;
            jrre <=32'h00000000;//nanndemo OK
        end
        //al
        else if(ctlbeq == 4'b0101) begin //bal//bgezal
            branchin <= fromreg1 <0 ?1:0;
            alout <= fromreg1 <0 ? 1:0;//true:next address is $ra
            jrre <=32'h00000000;//nanndemo OK
        end
        else if(ctlbeq == 4'b0110) begin //bal//bltzal
            branchin <= fromreg1 <0 ?1:0;
            alout <= fromreg1 >=0 ? 1:0;//true:next address is $ra
            jrre <=32'h00000000;//nanndemo OK
        end
        //al
        else if(ctlbeq == 4'b0111) begin //bal//bltz
            branchin <= fromreg1 >=0 ? 1:0;
            alout <= 1'b0;
            jrre <=32'h00000000;//nanndemo OK
        end
        else if(ctlbeq == 4'b1000) begin //R jr and jalr
            branchin <=1'b0;
            alout <= 1'b0;////////////////////////////////to pipeline 
            jrre <= fromreg1;//ALUCTL
        end
    end
endmodule


module beqjump(Rt,OP,FuncCode,out);
    input [4:0] Rt;//judge of bal.5bit
    input [5:0] OP;//judge of all branch 6bit
    input [5:0] FuncCode; //jr & jalr 
    output reg [3:0] out; //8char.

    always @(OP,FuncCode,Rt) begin
        if(OP == 6'b000100) begin //beq
            out=4'b0000;
        end
        else if(OP == 6'b000101) begin //bne
            out=4'b0001;
        end
        else if(OP == 6'b000110) begin //blez
            out=4'b0010;
        end
        else if(OP == 6'b000111) begin //bgtz
            out=4'b0011;
        end
        else if(OP == 6'b000001 && Rt == 5'b00001) begin //bal//bgez
            out=4'b0100;
        end
        else if(OP == 6'b000001 && Rt == 5'b10001) begin //bal//bgezal
            out=4'b0101;
        end
        else if(OP == 6'b000001 && Rt == 5'b10000) begin //bal//bltzal
            out=4'b0110;
        end
        else if(OP == 6'b000001 && Rt == 5'b00000) begin //bal//bltz
            out=4'b0111;
        end
        else if(OP == 6'b000000 && (FuncCode == 6'b001000 || FuncCode == 6'b001001)) begin //jr//jalr
            out=4'b1000;
        end
    end
endmodule


module iand(left,right,ans);
    input left;
    input right;
    output reg ans;
    always @(left,right) begin
        if(left& right) begin
            ans=1'b1;
        end
        else begin
            ans=1'b0;
        end
    end
endmodule 

module ALUControl(ALUOp,FuncCode,ALUCtl,jalrsig,shamtsig,rjump);
    input[1:0] ALUOp;
    input[5:0] FuncCode;
    output reg [3:0] ALUCtl;
    output reg shamtsig;//sll,srl,sra only

    //output reg sigreji;
    output reg jalrsig;
    output reg [1:0] rjump;

    always  @(FuncCode,ALUOp) begin
        if(ALUOp[1] ==1'b1 && FuncCode ==6'b000000) begin
                ALUCtl<=4'b0100;//sll0 //4
                shamtsig <=1'b1;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b000010) begin
                ALUCtl<=5;//srl2 //5
                shamtsig <=1'b1;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b000011) begin
                ALUCtl=8;//sra3 //8
                shamtsig <= 1'b1;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b000100) begin
                ALUCtl=9;//sllv4 //9
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b000110) begin
                ALUCtl=10;//srlv6 //10
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b000111) begin
                ALUCtl=11;//srav7 //11
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end


        //highorder"001xxx"
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b001000) begin
                //jr//OK
                ALUCtl<=14; //14
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b01;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b001001) begin
                //jalr//OK
                ALUCtl<=14; //14
                shamtsig <= 1'b0;
                jalrsig = 1'b1;
                rjump <=2'b01;
        end




        //high order"100xxx"
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100000) begin
                ALUCtl=2;//add32 //2
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100001) begin
                ALUCtl=2;//addu33 //2
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100010) begin
                ALUCtl=6;//sub34 //6
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100011) begin
                ALUCtl=6;//subu35 //6
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100100) begin
                ALUCtl=0;//and36 //0
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100101) begin
                ALUCtl=1;//or37 //1
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100110) begin
                ALUCtl=3;//xor//make38 //3
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b100111) begin
                ALUCtl=12;//nor39 //12
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        //high order  "101xxx"
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b101010) begin
                ALUCtl=7;//slt42 //7
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp[1] ==1'b1 && FuncCode ==6'b101011) begin
                ALUCtl=7;//sltu43 //7
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
        end
        else if(ALUOp == 2'b00) begin //lw,sw
            ALUCtl=2; //add //2
            shamtsig <= 1'b0;
            jalrsig <= 1'b0;
            rjump <=2'b00;
        end
        else begin
                ALUCtl <= 4'b1111;//15
                shamtsig <= 1'b0;
                jalrsig <= 1'b0;
                rjump <=2'b00;
                //ALUOp==01,
                //11 is judge [1] == 1x 
        end
    end
endmodule

//CTRL
module mainCTL(OP,
    RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,
    jalsig,
    IALUCtl,
    ALUOp,Jump,
    lwusig,
    SIZE
);
    //beq=Branch MemRead=lw MemWrite=sw
    input [31:26] OP; //head 6bit
    output reg RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
    output reg jalsig;
    output reg [3:0]IALUCtl;//4bit haba
    output reg [1:0] ALUOp,Jump;

    output reg lwusig;
    output reg [1:0] SIZE;


    always @(OP) begin
        if(OP == 6'b000000) begin //R //func //OK//13gyou
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b1;
            ALUOp    <=2'b10;
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
        else if(OP == 6'b000001) begin //bal
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b1;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//00->lwsw //10->R//01->X
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
        else if(OP == 6'b000010) begin //j
            RegDst   <=1'b0;
            Jump     <=2'b01;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//j->X
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
        else if(OP == 6'b000011) begin //jal 
            RegDst   <=1'b0;
            Jump     <=2'b01;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//jal J
            jalsig <=1'b1;///////////////////////OK
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
        end
        else if(OP == 6'b000100) begin //beq
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b1;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//beq 01->X
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b000101) begin //bne
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b1;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//bne 01->X
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b000110) begin //blez
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b1;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//blez 01->X
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b000111) begin //bgtz
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b1;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b0;
            RegWrite <=1'b0;
            ALUOp    <= 2'b01;//bgtz 01->X
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
        //highorder "001xxx"
        //beq,bne ,blez,bgtz is "bew_jumpctl.v"

        //I nomi CTRL kara tyokusetu ALU ni sijiwo dasu
        //because FuncCode is not use! I keisiki!
        else if(OP == 6'b001000) begin //addi 
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0010;//same add 2
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b001001) begin //addiu hugou
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0010;//same add 2 
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;    
        end        
        else if(OP == 6'b001010) begin //slti
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0111;//same slt 7
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b001011) begin //sltiu
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0111;//same slt 7    
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0; 
        end
        ////
        else if(OP == 6'b001100) begin //andi 0 kakutyou?????
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0000;//same and 0
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b001101) begin //ori 0 kakuryou????
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0001;//same or 1
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b001110) begin //xori 0 kakutyou?????
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b0011;//same xor 3
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end   
        ////
        else if(OP == 6'b001111) begin //lui
            RegDst   <=1'b1;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;//I
            RegWrite <=1'b1;
            ALUOp    <=2'b01;
            IALUCtl <= 4'b1101;//lui 13
            SIZE <= 2'b01;//X
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end

        /*
        //highorder "010xxx"
        //highorder "011xxx"
        */

        //highorder "100xxx"
        else if(OP == 6'b100000) begin //lb????????
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b1;
            MemtoReg <=1'b1;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;
            RegWrite <=1'b1;
            ALUOp    <=2'b00;//lw sw   
            IALUCtl <= 4'b1111;//15
            SIZE <=2'b10;
            lwusig <=1'b0;
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b100001) begin //lh??????
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b1;
            MemtoReg <=1'b1;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;
            RegWrite <=1'b1;
            ALUOp    <=2'b00;//lw sw
            IALUCtl <= 4'b1111;//15
            SIZE <=2'b01;
            lwusig <=1'b0;
            jalsig <= 1'b0;
        end
        //100,010
        else if(OP == 6'b100011) begin //lw //OK
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b1;
            MemtoReg <=1'b1;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;
            RegWrite <=1'b1;
            ALUOp    <=2'b00;//lw sw
            IALUCtl <= 4'b1111;//15
            SIZE <=2'b00;
            lwusig <=1'b0;
            jalsig <= 1'b0;
        end        
        else if(OP == 6'b100100) begin //lbu?????????
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b1;
            MemtoReg <=1'b1;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;
            RegWrite <=1'b1;
            ALUOp    <=2'b00;//lw sw
            IALUCtl <= 4'b1111;//15
            SIZE <=2'b10;
            lwusig <=1'b1;
            jalsig <= 1'b0;
        end
        else if(OP == 6'b100101) begin //lhu?????????
            RegDst   <=1'b0;
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b1;
            MemtoReg <=1'b1;
            MemWrite <=1'b0;
            ALUSrc   <=1'b1;
            RegWrite <=1'b1;
            ALUOp    <=2'b00;//lw sw
            IALUCtl <= 4'b1111;//15
            SIZE <=2'b01;
            lwusig <=1'b1;
            jalsig <= 1'b0;
        end

        //highorder "101xxx"
        else if(OP == 6'b101000) begin //sb
            RegDst   <=1'b0;//X
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;//X
            MemWrite <=1'b1;
            ALUSrc   <=1'b1;
            RegWrite <=1'b0;
            ALUOp    <=2'b00;
            IALUCtl <= 4'b1111;//15
            SIZE <=2'b10;
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
        else if(OP == 6'b101001) begin //sh
            RegDst   <=1'b0;//X
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;//X
            MemWrite <=1'b1;
            ALUSrc   <=1'b1;
            RegWrite <=1'b0;
            ALUOp    <=2'b00;
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b01;
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
        else if(OP == 6'b101011) begin //sw //OK
            RegDst   <=1'b0;//X
            Jump     <=2'b00;
            Branch   <=1'b0;
            MemRead  <=1'b0;
            MemtoReg <=1'b0;//X
            MemWrite <=1'b1;
            ALUSrc   <=1'b1;
            RegWrite <=1'b0;
            ALUOp    <=2'b00;
            IALUCtl <= 4'b1111;//15
            SIZE <= 2'b00;
            lwusig <=1'b0;//X
            jalsig <= 1'b0;
        end
    end
endmodule

module ctrlmux(
    input inRegDst,inBranch,inMemRead,inMemtoReg,inMemWrite,inALUSrc,inRegWrite,
    input injalsig,
    input [3:0] inIALUCtl,
    input [1:0] inALUOp,inJump,
    input inlwusig,
    input [1:0]inSIZE,

    input zerosig,

    output reg outRegDst,outBranch,outMemRead,outMemtoReg,outMemWrite,outALUSrc,outRegWrite,
    output reg outjalsig,
    output reg[3:0] outIALUCtl,
    output reg[1:0]outALUOp,outJump,
    output reg outlwusig,
    output reg [1:0] outSIZE
);

    always @(inRegDst,inBranch,inMemRead,inMemtoReg,inMemWrite,inALUSrc,inRegWrite,
            injalsig,inIALUCtl,
            inALUOp,inJump,
            inlwusig,inSIZE,zerosig) begin
        if(zerosig)begin
            outRegDst<=1'b0;
            outBranch<=1'b0;
            outMemRead<=1'b0;
            outMemtoReg<=1'b0;
            outMemWrite<=1'b0;
            outALUSrc<=1'b0;
            outRegWrite<=1'b0;

            outjalsig<=1'b0;
            outIALUCtl<=4'b0;
            outALUOp<=2'b01;//?????01 is ALUCTL don't output 
            outJump<=2'b00;
            outlwusig<=1'b0;
            outSIZE<=1'b0;
        end
        else begin
            outRegDst<=inRegDst;
            outBranch<=inBranch;
            outMemRead<=inMemRead;
            outMemtoReg<=inMemtoReg;
            outMemWrite<=inMemWrite;
            outALUSrc<=inALUSrc;
            outRegWrite<=inRegWrite;

            outjalsig<=injalsig;
            outIALUCtl<=inIALUCtl;
            outALUOp<=inALUOp;//?????01 is ALUCTL don't output 
            outJump<=inJump;
            outlwusig<=inlwusig;
            outSIZE<=inSIZE;
        end
    end
endmodule


//HARZARD
module hazard(muxzero,pcwrite,//out
    IF_IDwrite,//out
    ID_EX_MemRead,//out

    ID_EX_RegisterRt,//input
    ID_EX_RegisterRd,//input
    ID_EX_Regwrite,//input
    IF_ID_RegisterRs,//wire in
    IF_ID_RegisterRt,//wire in
    branch,//wire in
    EX_MEM_Regwrite,EX_MEM_Memread,
    EX_MEM_RegisterRt
    );
//in:4,out:3
   
    input ID_EX_MemRead,ID_EX_Regwrite;
    input [4:0] ID_EX_RegisterRt/*lw*/,ID_EX_RegisterRd/*Rd*/,IF_ID_RegisterRs,IF_ID_RegisterRt;
    input branch;

    input EX_MEM_Regwrite,EX_MEM_Memread;
    input [4:0] EX_MEM_RegisterRt;//==IFIDRegusterRs,Rt
    // lw-1st-beq -->lw -2st-beq

    output reg muxzero;
    output reg pcwrite,IF_IDwrite;


    always @(ID_EX_MemRead or ID_EX_Regwrite or ID_EX_RegisterRd or ID_EX_RegisterRt or 
    IF_ID_RegisterRs or IF_ID_RegisterRt or branch or EX_MEM_Regwrite or 
    EX_MEM_Memread or EX_MEM_RegisterRt) begin
        if(branch & ID_EX_Regwrite & ID_EX_MemRead &  ((ID_EX_RegisterRt == IF_ID_RegisterRs) || (ID_EX_RegisterRt == IF_ID_RegisterRt))) begin
            //lw-beq 1st
            //not forward
            pcwrite<= 1'b1;
            IF_IDwrite <= 1'b1;//Reset
            muxzero <= 1'b1;//1st
        end
        else if ((branch == 0) & ID_EX_Regwrite & ID_EX_MemRead &  ((ID_EX_RegisterRt == IF_ID_RegisterRs) || (ID_EX_RegisterRt == IF_ID_RegisterRt))) begin 
            //lw-add(addi),
            //lw-sw,
            //forward1
            pcwrite<= 1'b1;
            IF_IDwrite <= 1'b1;//Reset
            muxzero <= 1'b1;//1st
        end
        else if((branch == 0) & ID_EX_Regwrite & (ID_EX_MemRead == 0) & ((ID_EX_RegisterRt == IF_ID_RegisterRs) || (ID_EX_RegisterRt == IF_ID_RegisterRt))) begin
            //add-beq//addi-bew
            //for2
            pcwrite<= 1'b1;
            IF_IDwrite <= 1'b1;//Reset
            muxzero <= 1'b1;//1st
        end
        else if (branch & EX_MEM_Regwrite & EX_MEM_Memread & ((EX_MEM_RegisterRt == IF_ID_RegisterRs) || (EX_MEM_RegisterRt == IF_ID_RegisterRt))) begin 
            // lw-1st-beq -->lw -2st-beq
            //for2
            pcwrite<= 1'b1;
            IF_IDwrite <= 1'b1;//1:Reset
            muxzero <= 1'b1;
        end
        else begin 
            pcwrite<= 1'b0;
            IF_IDwrite <= 1'b0;
            muxzero <= 1'b0;
        end
    end
endmodule
//other way
    //assign muxzero = (ID_EX_MemRead & ( |(ID_EX_RegisterRt == IF_ID_RegisterRs) | (|(ID_EX_RegisterRt == IF_ID_RegisterRt))))? 1'b1: 1'b0;

//2bit left shift jump
module twobitljump(
        input[25:0] A,
        output[27:0] B
);
        assign B = (A<<2);
endmodule

//jhead32.v
module jhead(frompc,twei,out);
    input [3:0]frompc;
    input [27:0] twei;
    output [31:0] out;

    assign out[31:28]=frompc;
    assign out[27:0]=twei;

endmodule




`define  ZERO           1'b0           // Rename to Zero
`define  LOW            1'b0           // Rename to Zero
`define  HIGH           1'b1           // Rename to High

module rf32x32(
		// clock and reset
		clk,reset,//input
		// Inputs
		wr_n,//input regwrite
		rd1_addr, rd2_addr, //wire in 5bits
        wr_addr,//input "out of IDstage" 5bits
		data_in,//input "out of IDstage" 32bits 

		// Outputs
		data1_out, data2_out//wire out 32bits //to forC & forD
		);
   
   parameter data_width      = 32;
   parameter depth           = 32;
   parameter bit_width_depth = 5;  // ceil(log2(depth))
   parameter rst_mode        = 0;  // 0: asynchronously initializes the RAM
                                   // 1: synchronously

   //*** I/O declarations ***//
   input                          clk;       // clock
   input 			  reset;
   input                          wr_n;      // Write enable, active low
   input  [bit_width_depth-1 : 0] rd1_addr;  // Read0 address bus  
   input  [bit_width_depth-1 : 0] rd2_addr;  // Read1 address bus
   input  [bit_width_depth-1 : 0] wr_addr;   // Write address bus
   input       [data_width-1 : 0] data_in;   // Input data bus
   
   output      [data_width-1 : 0] data1_out; // Output data bus for read0
   output      [data_width-1 : 0] data2_out; // Output data bus for read1


   //*** wire declarations ***//
   wire 			  clk_inv;
   wire        [data_width-1 : 0] ram_data1_out;
   wire        [data_width-1 : 0] ram_data2_out;
   

   assign    clk_inv = ~clk;
   assign  data1_out = (|rd1_addr) ? ram_data1_out : {data_width{`ZERO}};
   assign  data2_out = (|rd2_addr) ? ram_data2_out : {data_width{`ZERO}};

   // Instance of DW_ram_2r_w_s_lat
   DW_ram_2r_w_s_dff #(data_width, depth, rst_mode)
      u_DW_ram_2r_w_s_dff(
             .clk(clk_inv), .rst_n(reset),//input "out of IDstage"
             .cs_n(`LOW), .wr_n(wr_n),//cs_n ???? //input "out of IDstage"//Regwrite
             .rd1_addr(rd1_addr), .rd2_addr(rd2_addr),//wire in 
             .wr_addr(wr_addr),//input "out of IDstage"//5bits 
             .data_in(data_in),//input "out of IDstage"//32bits data
             .data_rd1_out(ram_data1_out),//wire out 32bits data
             .data_rd2_out(ram_data2_out)//wire out 32bits data
             //input:6?(3)output:0,
             //wire in 2,wire out 2
      );

endmodule // rf32x32


////////////////////////////////////////////////////////////////////////////////
//
//       This confidential and proprietary software may be used only
//     as authorized by a licensing agreement from Synopsys Inc.
//     In the event of publication, the following notice is applicable:
//
//                    (C) COPYRIGHT 1999 - 2013 SYNOPSYS INC.
//                           ALL RIGHTS RESERVED
//
//       The entire notice above must be reproduced on all authorized
//     copies.
//
// AUTHOR:    Jay Zhu	Sept 22, 1999
//
// VERSION:   Simulation Architecture
//
// DesignWare_version: e321906e
// DesignWare_release: H-2013.03-DWBB_201303.4
//
////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------
// ABSTRACT:  Synch Write, Asynch Dual Read RAM (Flip-Flop Based)
//            (flip flop memory array)
//            legal range:  depth        [ 2 to 2048 ]
//            legal range:  data_width   [ 1 to 1024 ]
//            Input data: data_in[data_width-1:0]
//            Output data from read1: data_rd1_out[data_width-1:0]
//            Output data from read2: data_rd2_out[data_width-1:0]
//            Read1 Address: rd1_addr[addr_width-1:0]
//            Read2 Address: rd2_addr[addr_width-1:0]
//            Write Address: wr_addr[addr_width-1:0]
//            write enable (active low): wr_n
//            chip select (active low): cs_n
//            reset (active low): rst_n
//            clock:clk
//
//	MODIFIED:
//		092299	Jay Zhu		Rewrote for STAR91151
//              10/18/00  RPH       Rewrote accoding to new guidelines 
//                                  STAR 111067   
//              05/25/01  RJK       Rewritten again
//              2/18/09   RJK       Corrected default value for rst_mode
//				    STAR 9000294457
//----------------------------------------------------------------------


module DW_ram_2r_w_s_dff (clk, rst_n, cs_n, wr_n, rd1_addr, rd2_addr, 
			  wr_addr, data_in, data_rd1_out, data_rd2_out);

   parameter data_width = 4;
   parameter depth = 8;
   parameter rst_mode = 1;

`define DW_addr_width ((depth>256)?((depth>4096)?((depth>16384)?((depth>32768)?16:15):((depth>8192)?14:13)):((depth>1024)?((depth>2048)?12:11):((depth>512)?10:9))):((depth>16)?((depth>64)?((depth>128)?8:7):((depth>32)?6:5)):((depth>4)?((depth>8)?4:3):((depth>2)?2:1))))

   input [data_width-1:0] data_in;
   input [`DW_addr_width-1:0] rd1_addr;
   input [`DW_addr_width-1:0] rd2_addr;
   input [`DW_addr_width-1:0] wr_addr;
   input 		      wr_n;
   input 		   rst_n;
   input 		   cs_n;
   input 		   clk;

   output [data_width-1:0] data_rd1_out;
   output [data_width-1:0] data_rd2_out;

// synopsys translate_off
   wire [data_width-1:0]   data_in;
   reg [depth*data_width-1:0]    next_mem;
   reg [depth*data_width-1:0]    mem;
   wire [depth*data_width-1:0]   mem_mux1;
   wire [depth*data_width-1:0]   mem_mux2;
   
   wire 		   a_rst_n;
   

   
  
  initial begin : parameter_check
    integer param_err_flg;

    param_err_flg = 0;
    
	    
  
    if ( (data_width < 1) || (data_width > 2048) ) begin
      param_err_flg = 1;
      $display(
	"ERROR: %m :\n  Invalid value (%d) for parameter data_width (legal range: 1 to 2048)",
	data_width );
    end
  
    if ( (depth < 2) || (depth > 1024 ) ) begin
      param_err_flg = 1;
      $display(
	"ERROR: %m :\n  Invalid value (%d) for parameter depth (legal range: 2 to 1024 )",
	depth );
    end
  
    if ( (rst_mode < 0) || (rst_mode > 1 ) ) begin
      param_err_flg = 1;
      $display(
	"ERROR: %m :\n  Invalid value (%d) for parameter rst_mode (legal range: 0 to 1 )",
	rst_mode );
    end

  
    if ( param_err_flg == 1) begin
      $display(
        "%m :\n  Simulation aborted due to invalid parameter value(s)");
      $finish;
    end

  end // parameter_check
   
   assign mem_mux1 = mem >> (rd1_addr * data_width);

   assign data_rd1_out = ((rd1_addr ^ rd1_addr) !== {`DW_addr_width{1'b0}})? {data_width{1'bx}} : (
				(rd1_addr >= depth)? {data_width{1'b0}} :
				   mem_mux1[data_width-1 : 0] );
   
   assign mem_mux2 = mem >> (rd2_addr * data_width);

   assign data_rd2_out = ((rd2_addr ^ rd2_addr) !== {`DW_addr_width{1'b0}})? {data_width{1'bx}} : (
				(rd2_addr >= depth)? {data_width{1'b0}} :
				   mem_mux2[data_width-1 : 0] );
   
   assign a_rst_n = (rst_mode == 0)? rst_n : 1'b1;


  
   always @ (posedge clk or negedge a_rst_n) begin : registers
      integer i, j;
      
   
      next_mem = mem;

      if ((cs_n | wr_n) !== 1'b1) begin
      
	 if ((wr_addr ^ wr_addr) !== {`DW_addr_width{1'b0}}) begin
	    next_mem = {depth*data_width{1'bx}};	

	 end else begin
         
	    if ((wr_addr < depth) && ((wr_n | cs_n) !== 1'b1)) begin
	       for (i=0 ; i < data_width ; i=i+1) begin
		  j = wr_addr*data_width + i;
		  next_mem[j] = ((wr_n | cs_n) == 1'b0)? data_in[i] | 1'b0
					: mem[j];
	       end // for
	    end // if
	 end // if-else
      end // if   
   
   
      if (rst_n === 1'b0) begin
         mem <= {depth*data_width{1'b0}};
      end else begin
         if ( rst_n === 1'b1) begin
	    mem <= next_mem;
	 end else begin
	    mem <= {depth*data_width{1'bX}};
	 end
      end
   end // registers
   
    
  always @ (clk) begin : clk_monitor 
    if ( (clk !== 1'b0) && (clk !== 1'b1) && ($time > 0) )
      $display( "WARNING: %m :\n  at time = %t, detected unknown value, %b, on clk input.",
                $time, clk );
    end // clk_monitor 

// synopsys translate_on

`undef DW_addr_width
endmodule // DW_ram_2r_w_s_dff
