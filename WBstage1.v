module we1(
    input [31:0] fromplw,
    input[1:0] LASTSIZE,
    input signLW,
    //lwunsigned.v // out is wire "EDITLOAD"
    input [31:0]frompaddANS,
    input frompMEMTOREG,
    //mux.v(lwRmux) --> out:wire LASTJUDGE
    input [31:0]ALINKPC,
    input LINKSIG,
    output [31:0] GOREGDATA
    //mux(finmux) --> out: output GOREGDATA
    //input:7 -->figure same
    //output:1 -->figure same
    );
    //---------------------------------------------------------------------------

    wire [31:0] EDITLOAD; //from lwunsigned.v to lwRmux
    wire [31:0] LASTJUDGE;//from lwRmux to finmux 
    //-----------------------------------------------------------------------------

    lwunsign sizeoflw(
        .SIZE(LASTSIZE),.lwsig(signLW),.lwout(fromplw),
        .afterlw(EDITLOAD)//out : to lwRmux's "data1"
    );

    mux lwRmux(
        .data1(EDITLOAD),.data2(frompaddANS),.signal(frompMEMTOREG),
        .out(LASTJUDGE)// to finmux's data1
    );

    mux finmux(
        .data1(LASTJUDGE),.data2(ALINKPC),.signal(LINKSIG),
        .out(GOREGDATA)//------>to IDsatage's register data!
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