module ex1(
    EX_inIDEXREGISTERRT,EX_inIDEXREGISTERRD,
    EX_inREGDEST,
    //mux5.v //32register sitei
    EX_inIDEXREGISTERRS,//&EX_inIDEXREGISTERRT
    EX_inEXMEMREGISTERRDRT,EX_inMEMWBREGISTERRDRT,
    EX_inEXMEMREGWRITE,EX_inMEMWBREGWRITE,
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    EX_injalsig,EX_injalrsig,EX_inbalal,
    //allsum.v
    EX_infromRs,EX_infromRt,EX_infromMEMWB,EX_infromEXMEM,
    //forA,forB
    EX_infromshamt,
    EX_inshamtsignal,
    //shamt
    EX_iniformat,
    EX_inALUSRC,
    //EX_inalusrcmux 
    EX_infromALUctl,
    EX_infromIALUctl,
    //----------------------------------------------------------------------------------------------
    EX_outanswer,
    EX_outtoANDLINK,//-------------->to pipeline
    EX_outtopipereg5,
    EX_outfboutpipe
    //ALU
);
    input [4:0] EX_inIDEXREGISTERRT,EX_inIDEXREGISTERRD;
    input EX_inREGDEST;
    //mux5.v //32register sitei
    input [4:0] EX_inIDEXREGISTERRS;//&EX_inIDEXREGISTERRT
    input [4:0] EX_inEXMEMREGISTERRDRT,EX_inMEMWBREGISTERRDRT;
    input EX_inEXMEMREGWRITE,EX_inMEMWBREGWRITE;
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    input EX_injalsig,EX_injalrsig,EX_inbalal;
    //allsum.v
    input [31:0]EX_infromRs,EX_infromRt,EX_infromMEMWB,EX_infromEXMEM;
    //forA,forB
    input [4:0] EX_infromshamt;
    input EX_inshamtsignal;
    //shamt
    input [31:0] EX_iniformat;
    input EX_inALUSRC;
    //EX_inalusrcmux 
    input [3:0]EX_infromALUctl;
    input [3:0]EX_infromIALUctl;
    //-------------------------------------------------------------------------------

    output [31:0]EX_outanswer;
    output EX_outtoANDLINK;//-------------->to pipeline
    output [4:0] EX_outtopipereg5;
    output [31:0]EX_outfboutpipe;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*wire call*/
    wire [4:0] rtrd;//muxfive
    wire [1:0] forA,forB;//forwarding1
    wire anlink;
    wire muxsig;//allsum
    //wire [4:0]goline; //-> pipeline//direct!
    /*wire call*/

    /*wire call*/
    wire [31:0] faout,fbout;//forA out & forB out //////////////////////////////
    wire [31:0] toALUA; //shamtmux.v out
    wire [31:0] toALUB; //EX_inALUsrcmux out 
    //wire [31:0] EX_outanswerwire;//direct!
    /*wire call*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*fo to pipeline */
    //assign EX_outanswer = EX_outanswerwire;//direct!
    //assign alink  = anlink;
    //assign Register = goline;
    assign EX_outfboutpipe = fbout;
/////////////////////////////////////////////////

    muxfive muxreg5(
        .data1(EX_inIDEXREGISTERRT),
        .data2(EX_inIDEXREGISTERRD),
        .signal(EX_inREGDEST),
        .out(rtrd)
    );
    forwarding1 forwa1(
        .ID_EX_RegisterRs(EX_inIDEXREGISTERRS),.ID_EX_RegisterRt(EX_inIDEXREGISTERRT),
        .EX_MEM_RegisterR(EX_inEXMEMREGISTERRDRT),.MEM_WB_RegisterR(EX_inMEMWBREGISTERRDRT),
        .EX_MEM_RegWrite(EX_inEXMEMREGWRITE),.MEM_WB_RegWrite(EX_inMEMWBREGWRITE),
        .ForwardA(forA),.ForwardB(forB)//->forA,forB 2bit
    );
    allsum alls(
        .EX_injalsig(EX_injalsig),.EX_injalrsig(EX_injalrsig),.EX_inbalal(EX_inbalal),
        .andlink(EX_outtoANDLINK),.regictl(muxsig)
        //alink goes pipeline
        //muxsig is mux signal
    );
    mux31 mux31(
        .regi(rtrd),.signal(muxsig),
        .out(EX_outtopipereg5)//------------>to pipeline 
    );

    fmux forwardingA(
        .data1(EX_infromRs),.data2(EX_infromEXMEM),.data3(EX_infromMEMWB),.signal(forA),
        .out(faout)//->shamtmux.v 
    );
    fmux forwardingB(
        .data1(EX_infromRt),.data2(EX_infromEXMEM),.data3(EX_infromMEMWB),.signal(forB),
        .out(fbout)//=>EX_inALUSrcmux & => to pipeline
    );
    shmux shamtjudge(
        .shamtsig(EX_inshamtsignal),.shamt(EX_infromshamt),.fromR(faout),
        .out(toALUA)
    );
    //EX_inALUsrc
    mux arusrcmux(
        .data1(EX_iniformat),.data2(fbout),.signal(EX_inALUSRC),
        .out(toALUB)
    );
    mainALU ALU(
        .ALUctl(EX_infromALUctl),.IALUctl(EX_infromIALUctl),
        .A(toALUA),.B(toALUB),
        .ALUOut(EX_outanswer)//to pipeline 
    );

    /*
    to pipeline !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    EX_outanswer
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
    input EX_injalsig,EX_injalrsig,EX_inbalal,
    output reg andlink,
    output reg regictl
    );
    always @(EX_injalsig,EX_injalrsig,EX_inbalal) begin
        if(EX_injalsig | EX_inbalal) begin
            andlink <= 1'b1;
            regictl <= 1'b1; 
        end
        else if(EX_injalrsig) begin
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
