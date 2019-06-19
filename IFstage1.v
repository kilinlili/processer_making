module if1(
    input CLOCK,RESET,

    input WRITEPC,//from IDstage's harzard 
    //pc.v

    input [1:0]fromJUMP,fromRJUMP,
    input [31:0] fromJJAL,fromJRJALR,//--->jmlt sig 2
    //jmlt.v
    input fromPCSRC,
    //to mux.v
    //ALU4.v all wire 
    output [31:0] goifidpc4, //-------------->to pipeline 
    //assign this = addreturn
    input [31:0] BRANCHGO, //-->to mux.v
    
    output FLASHIF,//-------------------------->to pipeline

    input [31:0] IDTORD,//from top input
    output [31:0] IDTadd, //assign this = IDORD
    output [31:0] IADADD //assign this = toaddordermem //to top output
    //--------------------------------------------------------------------
    //input: 10 line //IFstage1.v's "input" is same my figure
    //output: 4 line //IFstage1.v's "output" is same my figure
    //(if ACKI ,+1 line output!)<------------------------------------(new!) not need! 


);

///////////////////////////////////////////////////////////////////////////////////////////////////
    wire [31:0] toaddordermem;//------> to adder & to order memory
    wire [31:0] jmltans;//----> to PC's pcin
    wire [31:0] addreturn;//------->to mux's data & to output goifidpc4
    wire [31:0] nextjudge;//--------> to jmlt.v
///////////////////////////////////////////////////////////////////////////////////////////////////



    fourALU fouradd(
        .A(toaddordermem),//wire in
        .ALUOut(addreturn)//wire out
    );
    assign goifidpc4 = addreturn;//--------> to pipeline 

    mux idmux(
        .data1(BRANCHGO),.data2(addreturn),
        .signal(fromPCSRC),
        .out(nextjudge)
    );

    mlt judgemulti(
        .rjump(fromJUMP),.jump(fromRJUMP),
        .jrjalr(fromJRJALR),.jjal(fromJJAL),.normal(nextjudge),
        .out(jmltans)
    );

    pc mypc(
        .clock(CLOCK),.reset(RESET),
        .pcin(jmltans),.pcwrite(WRITEPC),//pcin is "wire in" 
        .pcout(toaddordermem)//wire out--->to memo & to ALU
    );
    flushsum toflush(
        .A(fromPCSRC),.B(fromJUMP),.C(fromRJUMP),//<<<-----from input direct 
        .out(FLASHIF)//-------> to pipeline 
    );


    /*order memory*/
    assign IDTadd = IDTadd;//<---from pc to "out of IFstage"
    assign IADADD = toaddordermem;


endmodule


//userpc
module pc(clock,reset,pcin,pcwrite,pcout);

    input clock,reset;
    input [31:0] pcin;
    input 	pcwrite;
    output reg [31:0] pcout;
     
    reg [31:0] pcreg;

    always@(posedge clock or negedge reset or pcwrite) begin
        if(reset==1'b0) begin
	        pcout<=32'h00010000;
        end
        else if(clock == 1'b1) begin 
            pcout<=pcin;
            pcreg<=pcin;
        end
        else if(pcwrite == 1'b1) begin
            pcout<=pcreg;
        end
    end   
endmodule 

//jmlt.v
module mlt(rjump/*2*/,jump/*2*/,
    jrjalr/*32*/,jjal/*32*/,normal,/*32*/
    out
    );
    input [1:0] jump,rjump;
    input [31:0] jrjalr,jjal,normal;

    output reg [31:0] out;

    always@(jump,rjump,jrjalr,jjal,normal) begin
        if(jump == 2'b00 && rjump == 2'b01)begin
            out <= jrjalr;
        end
        else if(jump == 2'b01 && rjump == 2'b00) begin
            out <= jjal;
        end
        else if(jump == 2'b00 && rjump == 2'b00) begin
            out <= normal;
        end
        else begin
            
        end
    end
endmodule

//ALU4.v
module fourALU(A,ALUOut);
    input [31:0]A;
    output [31:0] ALUOut;

    assign ALUOut = A + 32'h00000004;

endmodule

//mux.v
module mux(data1,data2,signal,out);

    input[31:0]data1,data2;
    input signal;
    output [31:0] out;

    function [31:0]hoge;
        input[31:0]data1,data2;
        input signal;
        if(signal == 1'b1)hoge = data1;
        else hoge =data2;
    endfunction 

    assign out = hoge(data1,data2,signal);
endmodule

//flushsum.v
module flushsum(A,B,C,out);
    input A;
    input [1:0] B,C;
    output out;
    assign out = sum(A,B,C);
    function sum;
        input A;
        input [1:0] B,C;
        sum = A || (|B || |C);
    endfunction
endmodule


//order mem?????