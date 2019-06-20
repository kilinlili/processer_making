module we1(
    WB_infromplw,
    WB_inLASTSIZE,
    WB_insignLW,
    //lwunsigned.v // out is wire "EDITLOAD"
    WB_infrompaddANS,
    WB_infrompMEMTOREG,
    //mux.v(lwRmux) --> out:wire LASTJUDGE
    WB_inALINKPC,
    WB_inLINKSIG,
    //---------------------------------------------------------------------------------------------------------------
    WB_outGOREGDATA
    );

    input [31:0] WB_infromplw;
    input[1:0] WB_inLASTSIZE;
    input WB_insignLW;
    //lwunsigned.v // out is wire "EDITLOAD"
    input [31:0]WB_infrompaddANS;
    input WB_infrompMEMTOREG;
    //mux.v(lwRmux) --> out:wire LASTJUDGE
    input [31:0]WB_inALINKPC;
    input WB_inLINKSIG;
    output [31:0] WB_outGOREGDATA;
    //mux(finmux) --> out: output WB_inGOREGDATA
    //input:7 -->figure same
    //output:1 -->figure same
    //-------------------------------------------------------------------------------------------------------------------

    wire [31:0] EDITLOAD; //from lwunsigned.v to lwRmux
    wire [31:0] LASTJUDGE;//from lwRmux to finmux 
    //-------------------------------------------------------------------------------------------------------------------

    lwunsign sizeoflw(
        .SIZE(WB_inLASTSIZE),.lwsig(WB_insignLW),.lwout(WB_infromplw),
        .afterlw(EDITLOAD)//out : to lwRmux's "data1"
    );

    mux lwRmux(
        .data1(EDITLOAD),.data2(WB_infrompaddANS),.signal(WB_infrompMEMTOREG),
        .out(LASTJUDGE)// to finmux's data1
    );

    mux finmux(
        .data1(LASTJUDGE),.data2(WB_inALINKPC),.signal(WB_inLINKSIG),
        .out(WB_outGOREGDATA)//------>to IDsatage's register data!
    );


endmodule