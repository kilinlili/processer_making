module memwbpipe(
    input CLOCK,RESET,
    //
    input inlastMEMTOREG,inlastREGWRITE,
    input [1:0]inlastSIZE,
    input inlastLWSIG,
    /*CTRL 4lines */
    input [31:0] inlastlwans,//5
    input [31:0] inlastPC,//6
    input [31:0] inlastRform,//7
    input inlastandlinlsig,//8
    input [4:0] inlastwherereg,//9
    //input 9lines + CLOCK RESET

    //---------------------------------------->>>
    output reg FINMEMTOREG,FINREGWRITE,
    output reg [1:0] FINSIZE,
    output reg FINLWSIG,
    /*CTRL 4lines */
    output reg [31:0] FINlwans,//5
    output reg [31:0] FINPC,//6
    output reg [31:0] FINRform,//7
    output reg FINandlinlsig,//8
    output reg [4:0] FINwherereg//9
    //output 9lines 
);
    always@(posedge CLOCK or negedge RESET) begin
        if (RESET == 1'b0) begin
            FINMEMTOREG<=0;
            FINREGWRITE<=0;
            FINSIZE<=0;
            FINLWSIG<=0;
            /*CTRL 4lines */
            FINlwans<=0;//5
            FINPC<=0;//6
            FINRform<=0;//7
            FINandlinlsig<=0;//8
            FINwherereg<=0;//9
        end 
        else if(CLOCK == 1'b1) begin
            FINMEMTOREG<=inlastMEMTOREG;
            FINREGWRITE<=inlastREGWRITE;
            FINSIZE<=inlastSIZE;
            FINLWSIG<=inlastLWSIG;
            /*CTRL 4lines */
            FINlwans<=inlastlwans;//5
            FINPC<=inlastPC;//6
            FINRform<=inlastRform;//7
            FINandlinlsig<=inlastandlinlsig;//8
            FINwherereg<=inlastwherereg;//9
        end
    end





endmodule