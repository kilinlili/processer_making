module exmempipe(
    CLOCK,RESET,
    //
    EX_MEMinMemRead,EX_MEMinMemtoReg,EX_MEMinMemWrite,EX_MEMinRegWrite,
    EX_MEMinlwusig,
    EX_MEMinSIZE,
    /*CTRL 6lines*/
    EX_MEMinPCadd,//7
    /*PC address*/
    EX_MEMinALUans,EX_MEMinforb,//8,9
    EX_MEMinANDLINK,//10
    EX_MEMinREGISTER,//11

    //------------------------------------------------------>
    
    EX_MEMoutMemRead,EX_MEMoutMemtoReg,EX_MEMoutMemWrite,EX_MEMoutRegWrite,
    EX_MEMoutlwusig,
    EX_MEMoutSIZE,
    /*CTRL 6lines*/
    EX_MEMoutPCadd,//7
    /*PC address*/
    EX_MEMoutALUans,EX_MEMoutforb,//8,9
    EX_MEMoutANDLINK,//10
    EX_MEMoutREGISTER//11

);
    input CLOCK,RESET;
    //
    input EX_MEMinMemRead,EX_MEMinMemtoReg,EX_MEMinMemWrite,EX_MEMinRegWrite;
    input EX_MEMinlwusig;
    input [1:0]EX_MEMinSIZE;
    /*CTRL 6lines*/
    input [31:0] EX_MEMinPCadd;//7
    /*PC address*/
    input [31:0] EX_MEMinALUans,EX_MEMinforb;//8,9
    input EX_MEMinANDLINK;//10
    input [4:0] EX_MEMinREGISTER;//11

    //------------------------------------------------------>
    
    output reg EX_MEMoutMemRead,EX_MEMoutMemtoReg,EX_MEMoutMemWrite,EX_MEMoutRegWrite;
    output reg EX_MEMoutlwusig;
    output reg [1:0]EX_MEMoutSIZE;
    /*CTRL 6lines*/
    output reg [31:0] EX_MEMoutPCadd;//7
    /*PC address*/
    output reg [31:0] EX_MEMoutALUans,EX_MEMoutforb;//8,9
    output reg EX_MEMoutANDLINK;//10
    output reg [4:0] EX_MEMoutREGISTER;//11





    always@(posedge CLOCK or negedge RESET) begin
        if (RESET == 1'b0) begin
            EX_MEMoutSIZE <= 0;
            EX_MEMoutMemRead<= 0;
            EX_MEMoutMemtoReg<= 0;
            EX_MEMoutMemWrite<= 0;
            EX_MEMoutRegWrite<= 0;
            EX_MEMoutlwusig<= 0;
            /*CTRL 6lines*/
            EX_MEMoutPCadd<= 0;//7
            /*PC address*/
            EX_MEMoutALUans<= 0;
            EX_MEMoutforb<= 0;//8,9
            EX_MEMoutANDLINK<= 0;//10
            EX_MEMoutREGISTER<= 0;//11
        end 
        else if(CLOCK == 1'b1) begin
            EX_MEMoutSIZE <= EX_MEMinSIZE;
            EX_MEMoutMemRead<= EX_MEMinMemRead;
            EX_MEMoutMemtoReg<= EX_MEMinMemtoReg;
            EX_MEMoutMemWrite<= EX_MEMinMemWrite;
            EX_MEMoutRegWrite<= EX_MEMinRegWrite;
            EX_MEMoutlwusig<= EX_MEMinlwusig;
            /*CTRL 6lines*/
            EX_MEMoutPCadd<= EX_MEMinPCadd;//7
            /*PC address*/
            EX_MEMoutALUans<= EX_MEMinALUans;
            EX_MEMoutforb<= EX_MEMinforb;//8,9
            EX_MEMoutANDLINK<= EX_MEMinANDLINK;//10
            EX_MEMoutREGISTER<= EX_MEMinREGISTER;//11
        end
    end
endmodule