module ifidpipe(
    /*input*/
    CLOCK,RESET,
    IF_IDinpipeWRITE,
    IF_IDinPC4,
    IF_IDinORDER,
    IF_IDinFLASH,
    /*input*/
    /*output*/
    IF_IDoutTOADD,
    IF_IDoutTOMAINORDER
    /*output*/
    );
        /*input*/
    input CLOCK,RESET;
    input IF_IDinpipeWRITE;//stoll
    input [31:0]IF_IDinPC4;
    input [31:0]IF_IDinORDER;
    input IF_IDinFLASH;
    /*input*/
    /*output*/
    output reg [31:0] IF_IDoutTOADD,IF_IDoutTOMAINORDER;
    /*output*/


    always@(posedge CLOCK or negedge RESET) begin
        /*reset*/
        if(RESET == 1'b0) begin
            IF_IDoutTOADD <=0;
            IF_IDoutTOMAINORDER<=0;     
        end
        /*flush*/
        else if(IF_IDinFLASH ==1'b1) begin
            IF_IDoutTOADD <= 0;
            IF_IDoutTOMAINORDER <=0;
        end

        /*go or nop*/
        else if(CLOCK) begin
            if(IF_IDinpipeWRITE) begin // hoji
                IF_IDoutTOADD <= IF_IDoutTOADD;
                IF_IDoutTOMAINORDER<=IF_IDoutTOMAINORDER;
            end
            else begin
                IF_IDoutTOADD <= IF_IDinPC4;
                IF_IDoutTOMAINORDER <= IF_IDinORDER;

            end
        end
    end
endmodule