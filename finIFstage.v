module if1(

    CLOCK,RESET,
    IF_inWRITEPC,
    IF_infromJUMP,IF_infromRJUMP,
    IF_infromJJAL,IF_infromJRJALR,//--->jmlt sig 2
    IF_infromPCSRC,
    IF_inBRANCHGO,//-->to mux.v
    //---------------------------------------------------------------------------------------------------------------
    IF_outIADADD, //------------------------to top output
    IF_outFLASHIF,//-------------------------->to pipeline
    IF_outgoifidpc4 //-------------->to pipeline 
);

    input CLOCK,RESET;
    input IF_inWRITEPC;//from IDstage's harzard 
    //pc.v
    input [1:0]IF_infromJUMP,IF_infromRJUMP;
    input [31:0] IF_infromJJAL,IF_infromJRJALR;//--->jmlt sig 2
    input IF_infromPCSRC;
    //to mux.v
    //ALU4.v all wire 
    //assign this = addreturn
    input [31:0] IF_inBRANCHGO; //-->to mux.v
    //---------------------------------------------------------------------------------------------------------------

    output [31:0] IF_outIADADD; //to top output
    output IF_outFLASHIF;//-------------------------->to pipeline
    output [31:0] IF_outgoifidpc4; //-------------->to pipeline 
    //input: 9 line //IFstage1.v's "input" is same my figure
    //output: 3 line //IFstage1.v's "output" is same my figure

///////////////////////////////////////////////////////////////////////////////////////////////////
    wire [31:0] toaddordermem;//------> to adder & to order memory
    wire [31:0] jmltans;//----> to PC's pcin
    wire [31:0] addreturn;//------->to mux's data & to output IF_outgoifidpc4
    wire [31:0] nextjudge;//--------> to jmlt.v
///////////////////////////////////////////////////////////////////////////////////////////////////



    fourALU fouradd(
        .A(toaddordermem),//wire in
        .ALUOut(addreturn)//wire out
    );
    assign IF_outgoifidpc4 = addreturn;//--------> to pipeline 

    mux idmux(
        .data1(IF_inBRANCHGO),.data2(addreturn),
        .signal(IF_infromPCSRC),
        .out(nextjudge)
    );

    mlt judgemulti(
        .rjump(IF_infromJUMP),.jump(IF_infromRJUMP),
        .jrjalr(IF_infromJRJALR),.jjal(IF_infromJJAL),.normal(nextjudge),
        .out(jmltans)
    );

    pc mypc(
        .clock(CLOCK),.reset(RESET),
        .pcin(jmltans),.pcwrite(IF_inWRITEPC),//pcin is "wire in" 
        .pcout(toaddordermem)//wire out--->to memo & to ALU
    );
    flushsum toflush(
        .A(IF_infromPCSRC),.B(IF_infromJUMP),.C(IF_infromRJUMP),//<<<-----from input direct 
        .out(IF_outFLASHIF)//-------> to pipeline 
    );

    /*order memory*/
    assign IF_outIADADD = toaddordermem;//to output top's order  memory 


endmodule


//order mem?????