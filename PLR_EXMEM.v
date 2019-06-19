module exmempipe(
    input CLOCK,RESET,
    //
    input fromMemRead,fromMemtoReg,fromMemWrite,fromRegWrite,
    input fromlwusig,
    input [1:0]fromSIZE,
    /*CTRL 6lines*/
    input [31:0] fromPCadd,//7
    /*PC address*/
    input [31:0] fromALUans,fromforb,//8,9
    input fromANDLINK,//10
    input [4:0] fromREGISTER,//11

    //------------------------------------------------------>
    
    output reg GOMemRead,GOMemtoReg,GOMemWrite,GORegWrite,
    output reg GOlwusig,
    output reg [1:0]GOSIZE,
    /*CTRL 6lines*/
    output reg [31:0] GOPCadd,//7
    /*PC address*/
    output reg [31:0] GOALUans,GOforb,//8,9
    output reg GOANDLINK,//10
    output reg [4:0] GOREGISTER//11

);
    always@(posedge CLOCK or negedge RESET) begin
        if (RESET == 1'b0) begin
            GOSIZE <= 0;
            GOMemRead<= 0;
            GOMemtoReg<= 0;
            GOMemWrite<= 0;
            GORegWrite<= 0;
            GOlwusig<= 0;
            /*CTRL 6lines*/
            GOPCadd<= 0;//7
            /*PC address*/
            GOALUans<= 0;
            GOforb<= 0;//8,9
            GOANDLINK<= 0;//10
            GOREGISTER<= 0;//11
        end 
        else if(CLOCK == 1'b1) begin

            GOMemRead<= fromMemRead;
            GOMemtoReg<= fromMemtoReg;
            GOMemWrite<= fromMemWrite;
            GORegWrite<= fromRegWrite;
            GOlwusig<= fromlwusig;
            /*CTRL 6lines*/
            GOPCadd<= fromPCadd;//7
            /*PC address*/
            GOALUans<= fromALUans;
            GOforb<= fromforb;//8,9
            GOANDLINK<= fromANDLINK;//10
            GOREGISTER<= fromREGISTER;//11
        end
    end
endmodule