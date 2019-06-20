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


module lwunsign(SIZE,lwsig,lwout,afterlw);
    input [1:0] SIZE;
    input lwsig;
    input [31:0]lwout;
    output [31:0]afterlw;

    assign afterlw = lwjudge(SIZE,lwsig,lwout);
    function [31:0] lwjudge;
        input [1:0] SIZE;
        input lwsig;
        input [31:0]lwout;

        if(SIZE == 2'b00 && lwsig == 1'b0)begin
            lwjudge = lwout;//lw
        end
        else if(SIZE == 2'b01 && lwsig == 1'b0)begin//lh -- signEx
            if(lwout[15] == 1'b1) begin
                lwjudge[31:16] =  16'hffff;
                lwjudge[15:0] = lwout[15:0];
            end
            else begin
                lwjudge[31:16] =  16'h0000;
                lwjudge[15:0] = lwout[15:0];
            end
        end
        else if(SIZE == 2'b10 && lwsig == 1'b0)begin//lb -- signEx
            if(lwout[7] == 1'b1) begin
                lwjudge[31:8] =  24'hffffff;
                lwjudge[7:0] = lwout[7:0];
            end
            else begin
                lwjudge[31:8] =  24'h000000;
                lwjudge[7:0] = lwout[7:0];
            end    
        end

        else if(SIZE == 2'b01 && lwsig == 1'b1)begin//lhu --0Ex
            lwjudge[31:16] =  16'h0000;
            lwjudge[15:0] = lwout[15:0];
        end
        else if(SIZE == 2'b10 && lwsig == 1'b1)begin//lbu -- 0Ex
            lwjudge[31:8] =  24'h000000;
            lwjudge[7:0] = lwout[7:0];
        end
    endfunction
endmodule

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

///////////////