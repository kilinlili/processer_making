module ifidpipe(
    /*input*/
    input CLOCK,RESET,
    input IFIDWRITE,//flush1
    input [31:0]FROMIFPC4,
    input [31:0]FROMORDER,
    input IFFLASH,
    /*input*/
    /*output*/
    output reg [31:0] TOADD,TOMAINORDER
    /*output*/

    );

    always@(posedge CLOCK or negedge RESET) begin
        /*reset*/
        if(RESET == 1'b0) begin
            TOADD <=0;
            TOMAINORDER<=0;     
        end
        /*flush*/
        else if(IFFLASH ==1'b1) begin
            TOADD <= 0;
            TOMAINORDER <=0;
        end

        /*go or nop*/
        else if(CLOCK) begin
            if(IFIDWRITE) begin // hoji
                TOADD <= TOADD;
                TOMAINORDER<=TOMAINORDER;
            end
            else begin
                TOADD <= FROMIFPC4;
                TOMAINORDER <= FROMORDER;

            end
        end
    end
endmodule