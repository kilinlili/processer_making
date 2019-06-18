module ex1(
    input [4:0] ID_EX_RegisterRt,ID_EX_RegisterRd,
    input Regdest,
    //mux5.v //32register sitei
    input [4:0] ID_EX_RegisterRs,//&ID_EX_RegisterRt 
    input [4:0] EX_MEM_RegisterRdRt,MEM_WB_RegisterRdRt,
    input EX_MEM_RegWrite,MEM_WB_RegWrite,
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    input jalsig,jalrsig,balal,
    //allsum.v
    input [31:0]fromRs,fromRt,fromMEMWB,fromEXMEM,
    //forA,forB
    input [4:0] fromshamt,
    input shamtsignal,
    //shamt
    input [31:0] iformat,
    input ALUsrc,
    //alusrcmux 
    input [3:0]fromALUctl,
    input [3:0]fromIALUctl,
    ////////////////////////////////////
    output [31:0]answer,
    output alink,
    output [4:0] Register,
    output [31:0]fboutpipe
    //ALU
);

///////////////////////////////////////////////////////////////////////////
    /*wire call*/
    wire [4:0] rtrd;//muxfive
    wire [1:0] forA,forB;//forwarding1
    wire anlink,muxsig;//allsum//alink -> pipeline//////////////////////////////
    wire [4:0]goline; //-> pipeline/////////////////////////////////////////////
    /*wire call*/

    /*wire call*/
    wire [31:0] faout,fbout;//forA out & forB out //////////////////////////////
    wire [31:0] toALUA; //shamtmux.v out
    wire [31:0] toALUB; //ALUsrcmux out 
    wire [31:0] answerwire;/////////////////////////////////////////////////////
    /*wire call*/

////////////////////////////////////////////////////////////////////////////

/*fo to pipeline */
    assign answer = answerwire;
    assign alink  = anlink;
    assign Register = goline;
    assign fboutpipe = fbout;
/////////////////////////////////////////////////

    muxfive muxreg5(
        .data1(ID_EX_RegisterRt),
        .data2(ID_EX_RegisterRd),
        .signal(Regdest),
        .out(rtrd)
    );
    forwarding1 forwa1(
        .ID_EX_RegisterRs(ID_EX_RegisterRs),.ID_EX_RegisterRt(ID_EX_RegisterRt),
        .EX_MEM_RegisterR(EX_MEM_RegisterRdRt),.MEM_WB_RegisterR(MEM_WB_RegisterRdRt),
        .EX_MEM_RegWrite(EX_MEM_RegWrite),.MEM_WB_RegWrite(MEM_WB_RegWrite),
        .ForwardA(forA),.ForwardB(forB)//->forA,forB 2bit
    );
    allsum alls(
        .jalsig(jalsig),.jalrsig(jalrsig),.balal(balal),
        .andlink(anlink),.regictl(muxsig)
        //alink goes pipeline
        //muxsig is mux signal
    );
    mux31 mux31(
        .regi(rtrd),.signal(muxsig),.out(goline)
    );

    fmux forwardingA(
        .data1(fromRs),.data2(fromEXMEM),.data3(fromMEMWB),.signal(forA),
        .out(faout)//->shamtmux.v 
    );
    fmux forwardingB(
        .data1(fromRt),.data2(fromEXMEM),.data3(fromMEMWB),.signal(forB),
        .out(fbout)//=>ALUSrcmux & => to pipeline
    );
    shmux shamtjudge(
        .shamtsig(shamtsignal),.shamt(fromshamt),.fromR(faout),
        .out(toALUA)
    );
    //ALUsrc
    mux arusrcmux(
        .data1(iformat),.data2(fbout),.signal(ALUsrc),
        .out(toALUB)
    );
    mainALU ALU(
        .ALUctl(fromALUctl),.IALUctl(fromIALUctl),
        .A(toALUA),.B(toALUB),
        .ALUOut(answerwire)//to pipeline 
    );

    /*
    to pipeline !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    answer
    alink
    muxsig
    fbout
    */




endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//mux5.v
module muxfive(data1,data2,signal,out);
    input[4:0]data1,data2;
    input signal;
    output [4:0] out;

    function [4:0]hoge;
        input[4:0]data1,data2;
        input signal;
        if(signal == 1'b1)hoge = data2;
        else hoge =data1;
    endfunction 

    assign out = hoge(data1,data2,signal);
endmodule

//FORWARDING_UNIT1.v
module forwarding1(
    input [4:0] ID_EX_RegisterRs,ID_EX_RegisterRt,
    input [4:0] EX_MEM_RegisterR,MEM_WB_RegisterR,
    input  EX_MEM_RegWrite,MEM_WB_RegWrite,
    output [1:0] ForwardA,ForwardB
    );
    assign ForwardA=forward(EX_MEM_RegWrite,MEM_WB_RegWrite,EX_MEM_RegisterR,MEM_WB_RegisterR,ID_EX_RegisterRs);
    assign ForwardB=forward(EX_MEM_RegWrite,MEM_WB_RegWrite,EX_MEM_RegisterR,MEM_WB_RegisterR,ID_EX_RegisterRt);
    function [1:0] forward;
        input [4:0] ID_EX_RegisterR;//2
        input [4:0] EX_MEM_RegisterR,MEM_WB_RegisterR;//2,2
        input  MEM_WB_RegWrite,EX_MEM_RegWrite;
        
        if(EX_MEM_RegWrite & (|EX_MEM_RegisterR != 0) & |(EX_MEM_RegisterR == ID_EX_RegisterR))
            //add-sw
            //add-add//add-lw
            forward = 2'b10;
        else if(MEM_WB_RegWrite & (|MEM_WB_RegisterR != 0) &(MEM_WB_RegisterR == ID_EX_RegisterR) &
         ~(EX_MEM_RegWrite & (|EX_MEM_RegisterR != 0) & |(EX_MEM_RegisterR == ID_EX_RegisterR)))
            //lw->sw
            //add-~~~-add
            forward = 2'b01;
        else
            forward = 2'b00;//ID/EX
    endfunction
endmodule

//allsum.v
module  allsum(
    input jalsig,jalrsig,balal,
    output reg andlink,
    output reg regictl
    );
    always @(jalsig,jalrsig,balal) begin
        if(jalsig | balal) begin
            andlink <= 1'b1;
            regictl <= 1'b1; 
        end
        else if(jalrsig) begin
            andlink <=1'b1;
            regictl <=1'b0;
        end
    end//OK
endmodule

//mux31.v
module mux31(
    input[4:0]regi,
    input signal,
    output [4:0] out
);

    function [4:0]hoge;
        input[4:0]regi;
        input signal;
        if(signal == 1'b1)hoge = 5'b11111;
        else hoge =regi;
    endfunction 

    assign out = hoge(regi,signal);
endmodule

//fdmux.v ->>>>forA.forB
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

//forA_faout -> shamt5bit(32 expand) 
module shmux(
    input shamtsig,
    input [4:0] shamt,
    input [31:0] fromR,
    output reg [31:0] out
);

    always @(shamtsig,shamt,fromR) begin
        if(shamtsig == 1'b1)begin
            out <= 26'b0 + shamt;
        end
        else begin
            out <= fromR;
        end
    end
endmodule

//mux 32bit 32bit in 32bit out
module mux(
    input[31:0]data1,data2,
    input signal,
    output [31:0] out
);

    function [31:0]hoge;
        input[31:0]data1,data2;
        input signal;
        if(signal == 1'b1)hoge = data1;
        else hoge =data2;
    endfunction 

    assign out = hoge(data1,data2,signal);
endmodule


//ALU.v
module mainALU (
    input [3:0]ALUctl,//IALUctl;????
    input [3:0]IALUctl,
    input [31:0]A,B,
    output reg [31:0] ALUOut,
    output Zero
);

    assign Zero = (ALUOut == 0);

    always @(ALUctl,A,B) begin
        if(ALUctl ==  4'b0000) begin
            ALUOut <= A&B; //and0
        end
        else if(ALUctl == 4'b0001) begin
            ALUOut <= A|B; //or1
        end
        else if(ALUctl == 4'b0010) begin
            ALUOut <= A+B; //add//addu//lw//sw //2 
        end
        else if(ALUctl == 4'b0011) begin
            ALUOut <= A^B; //xor3
        end


        else if(ALUctl == 4'b0100) begin
            ALUOut <= B<<A;//sll //sll Rd Rt << shamt //logic 
        end
        else if(ALUctl == 4'b0101) begin
            ALUOut <= B>>A;//srl //srl Rd Rt >> shamt  //logic
        end
        else if(ALUctl == 4'b0110) begin
            ALUOut <= A-B; //sub6
        end
        else if(ALUctl == 4'b0111) begin
            ALUOut <= A<B? 1:0; //slt//sltu7
        end
        else if(ALUctl == 4'b1000) begin
            ALUOut <= $signed(B)>>>$signed(A) ;//sra//?srav8 //arithmetic
        end
        else if(ALUctl == 4'b1001) begin
            ALUOut <= B<<A;//sllv9  //sllv Rd Rt << Rs //A = Rs of 32 nara OK
        end
        else if(ALUctl == 4'b1010) begin
            ALUOut <= B>>A;//srlv10 //srav Rd Rt >> Rs //A = Rs of 32 nara OK
        end
        else if(ALUctl == 4'b1011) begin
            ALUOut <=$signed(B)>>$signed(A);//srav11//arithmetic//A = Rs 32 nara OK 
        end
        else if(ALUctl == 4'b1100) begin
            ALUOut <= ~(A|B); //nor12
        end
        else begin
            //ALUOut <= 0;//????131415
        end
    end

    //Ikeisiki 
    always @(IALUctl,A,B) begin
        if(IALUctl == 4'b0010) begin
            //addi //addiu 2
            ALUOut <= A+B;
        end
        else if(IALUctl == 4'b0111)begin
            //slti//sltiu 7
            ALUOut <= A<B? 1:0;
        end
        //0kakutyou//////////////////////////////////////
        else if(IALUctl == 4'b0000)begin
            //andi 0
            ALUOut <= A&(16'b0 + B[15:0]);
        end
        else if(IALUctl == 4'b0001)begin
            //ori 1 
            ALUOut <= A|(16'b0 + B[15:0]);
        end
        else if(IALUctl == 4'b0011)begin
            //xori 3
            ALUOut <= A^(16'b0 + B[15:0]);
        end

        else if(ALUctl == 4'b1101) begin
            //I lui 13 ; B[15:0]Imm ////////////////////??????
            ALUOut <= B[15:0] + 16'b0;
        end
        else begin
            //ALUOut <= 0; //456,8910 ~ 15
        end
    end
endmodule


//wireはassignで代入
