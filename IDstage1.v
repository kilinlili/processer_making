module id1(
    input  [31:0] thirtytwo,//from IFIDpipeline
    output [4:0] toEXRd,
    output [4:0] toEXRs,
    output [4:0] toEXRt,
    output [4:0] toshamt,
    /*32bits main data*/

    input EXMEMRegWrite,EXMEMMemread,
    input MEMWBRegWrite,MEMWBMemread,
    input [4:0]EXMEMRegisterRdRt,MEMWBRegisterRt,
    ///////////////////////////////
    //input [4:0]IFIDRegisterRs,IFIDRegisterRt
    //forwarding_unit2
    output Immout,
    //16_32.v
    input [31:0] WBdata,MEMdata,//& fromRs(32),fromRt(32)
    output [31:0] fromC,fromD,
    //forwardC & forwardD

    /*register*/
    input [4:0] writeregi,//& wire IFIDRs & wire IFIDRt
    input [31:0] writeregidata,
    input WBRegwrite,
    //out:wire fromRegRs,fromRegRt
    /*register*/

    input [31:0] tobranchaddB,
    output [31:0] branchaddanswer,
    //from 2bitleft & from IFID pipeline(this!)  
    //out:IFstage's mux;
    //Is this "output"?????????????????????????????????????????????????????????

    output balandlink,//to pipeline
    output [31:0] beqtojrjalr32,
    //main_beq.v
    //beq_jumpCTL.v is all wire 
    output toIFpcsrc,//to IFstage 
    //iand

    output [3:0]ALUctltopipe,
    output ALUopshamtsig,
    output ALUopjalrsig,
    output /*wire ??*/[1:0] ALUoprjump,
    /*
    input:wire fromCTRL's ALUop[1:0]
    */
    //ALUCTL.v

    output [1:0] toIFjump
    //CTRL.v // almost wire


);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    wire [5:0]IFIDop = thirtytwo[31:26];
    wire [4:0]IFIDRd = thirtytwo[15:11];//-->to for2 & to pipeline(output toEXRd) 
    wire [4:0]IFIDRs = thirtytwo[25:21];//-->to for2 & to pipeline(output toEXRs) & register
    wire [4:0]IFIDRt = thirtytwo[20:16];//-->to for2 & to pipeline(output toEXRt) & register 
    wire [4:0]IFIDsh = thirtytwo[10: 6];//-->to pipeline --> EXstage shamtmux --> ALU
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

    /*CTRL wires*/
    wire CTRLRegDst,CTRLBranch,CTRLMemRead,CTRLMemtoReg,CTRLMemWrite,CTRLALUSrc,CTRLRegWrite;
    wire CTRLjalsig;
    wire [3:0] CTRLIALUCtl;
    wire [1:0] CTRLALUOp,CTRLJump;
    wire CTRLlwusig;
    wire [1:0] CTRLSIZE;
    /*CTRL wires*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    assign toEXRd = thirtytwo[15:11];//----->to pipeline 
    assign toEXRs = thirtytwo[25:21];//----->to pipeline
    assign toEXRt = thirtytwo[20:16];//----->to pipeline //2 line? --> 1line change OK
    assign toshamt= thirtytwo[10:6]; //----->to pipeline  // or IFIDsh OK

    forwarding2 forward2(
        .ForwardC(forc),.ForwardD(ford),//to forwardc & fowardd
        .EX_MEM_RegWrite(EXMEMRegWrite),.EX_MEM_Memread(EXMEMMemread),
        .MEM_WB_RegWrite(MEMWBRegWrite),.MEM_WB_Memread(MEMWBMemread),
        .EX_MEM_RegisterR(EXMEMRegisterRdRt),.MEM_WB_RegisterRt(MEMWBRegisterRt),
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
        .data1(tobranchaddA),.data2(tobranchaddB),.adder_out(branchaddanswer)///////////////////////////////////////////////////////???????????????????????????????????????
    );

    beqjump beqjumpctrl(
        .Rt(IFIDRt),.OP(IFIDop),.FuncCode(aluopandbeqjumpfunccode)/*6bit*/,
        .out(beqjumpcode)/*4bit*/
    );

    mainbeq i_mainbeq(
        .fromreg1(Ctobeqandpipe)/*32*/,.fromreg2(Dtobeqandpipe)/*32bit*/,.ctlbeq(beqjumpcode)/*6bit in*/,
        .branchin(tobeqand),.alout(balandlink),//wire:tobeqand -> iand(R)//output balandlink
        .jrre(beqtojrjalr32)////////////////////////////////////////////////////////?????????????????????????????????????????????????????????????/////32bit jump saki data
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


    ALUControl iALU(
        .ALUOp(),.FuncCode(aluopandbeqjumpfunccode),//wire input
        .ALUCtl(ALUctltopipe),.jalrsig(ALUopjalrsig),.shamtsig(ALUopshamtsig),//-------------------------------------------------output; to pipeline 
        .rjump(ALUoprjump)//----------------------------------------------------------------------------------------output ->IFstage
    );
    iand nameand(
        .left(),.right(tobeqand),
        .ans(toIFpcsrc)//output OK ??
    );




endmodule








///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module forwarding2(ForwardC,ForwardD,
    EX_MEM_RegWrite,EX_MEM_Memread,
    MEM_WB_RegWrite,MEM_WB_Memread,
    EX_MEM_RegisterR,MEM_WB_RegisterRt,
    IF_ID_RegisterRs,IF_ID_RegisterRt
);

    input  EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite,MEM_WB_Memread;
    input [4:0] EX_MEM_RegisterR,MEM_WB_RegisterRt/*lw*/;
    input [4:0] IF_ID_RegisterRs,IF_ID_RegisterRt; 

    output [1:0] ForwardC,ForwardD;

    assign ForwardC=forward(EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite,MEM_WB_Memread, EX_MEM_RegisterR/*RdRt*/,MEM_WB_RegisterRt,IF_ID_RegisterRs);
    assign ForwardD=forward(EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite,MEM_WB_Memread, EX_MEM_RegisterR/*RdRt*/,MEM_WB_RegisterRt,IF_ID_RegisterRt);

    function [1:0] forward;
        input  EX_MEM_RegWrite,EX_MEM_Memread,MEM_WB_RegWrite,MEM_WB_Memread;
        input [4:0] EX_MEM_RegisterR,MEM_WB_RegisterRt/*lw*/;
        input [4:0] IF_ID_RegisterR;

        if(EX_MEM_RegWrite & (EX_MEM_Memread==0) & ((EX_MEM_RegisterR == IF_ID_RegisterR))) 
            //add-1st-beq OK//addi-1st-beq OK
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

