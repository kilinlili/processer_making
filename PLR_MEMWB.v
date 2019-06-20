module memwbpipe(
    CLOCK,RESET,
    //
    MEM_WBinlastMEMTOREG,MEM_WBinlastREGWRITE,
    MEM_WBinlastSIZE,
    MEM_WBinlastLWSIG,
    /*CTRL 4lines */
    MEM_WBinlastlwans,//5
    MEM_WBinlastPC,//6
    MEM_WBinlastRform,//7
    MEM_WBinlastandlinlsig,//8
    MEM_WBinlastwherereg,//9
    //input 9lines + CLOCK RESET

    //---------------------------------------->>>
    MEM_WBoutMEMTOREG,MEM_WBoutREGWRITE,
    MEM_WBoutSIZE,
    MEM_WBoutLWSIG,
    /*CTRL 4lines */
    MEM_WBoutlwans,//5
    MEM_WBoutPC,//6
    MEM_WBoutRform,//7
    MEM_WBoutandlinlsig,//8
    MEM_WBoutwherereg//9
    //output 9lines 
);
    input CLOCK,RESET;
    //
    input MEM_WBinlastMEMTOREG,MEM_WBinlastREGWRITE;
    input [1:0]MEM_WBinlastSIZE;
    input MEM_WBinlastLWSIG;
    /*CTRL 4lines */
    input [31:0] MEM_WBinlastlwans;//5
    input [31:0] MEM_WBinlastPC;//6
    input [31:0] MEM_WBinlastRform;//7
    input MEM_WBinlastandlinlsig;//8
    input [4:0] MEM_WBinlastwherereg;//9
    //input 9lines + CLOCK RESET

    //---------------------------------------->>>
    output reg MEM_WBoutMEMTOREG,MEM_WBoutREGWRITE;
    output reg [1:0] MEM_WBoutSIZE;
    output reg MEM_WBoutLWSIG;
    /*CTRL 4lines */
    output reg [31:0] MEM_WBoutlwans;//5
    output reg [31:0] MEM_WBoutPC;//6
    output reg [31:0] MEM_WBoutRform;//7
    output reg MEM_WBoutandlinlsig;//8
    output reg [4:0] MEM_WBoutwherereg;//9


    always@(posedge CLOCK or negedge RESET) begin
        if (RESET == 1'b0) begin
            MEM_WBoutMEMTOREG<=0;
            MEM_WBoutREGWRITE<=0;
            MEM_WBoutSIZE<=0;
            MEM_WBoutLWSIG<=0;
            /*CTRL 4lines */
            MEM_WBoutlwans<=0;//5
            MEM_WBoutPC<=0;//6
            MEM_WBoutRform<=0;//7
            MEM_WBoutandlinlsig<=0;//8
            MEM_WBoutwherereg<=0;//9
        end 
        else if(CLOCK == 1'b1) begin
            MEM_WBoutMEMTOREG<=MEM_WBinlastMEMTOREG;
            MEM_WBoutREGWRITE<=MEM_WBinlastREGWRITE;
            MEM_WBoutSIZE<=MEM_WBinlastSIZE;
            MEM_WBoutLWSIG<=MEM_WBinlastLWSIG;
            /*CTRL 4lines */
            MEM_WBoutlwans<=MEM_WBinlastlwans;//5
            MEM_WBoutPC<=MEM_WBinlastPC;//6
            MEM_WBoutRform<=MEM_WBinlastRform;//7
            MEM_WBoutandlinlsig<=MEM_WBinlastandlinlsig;//8
            MEM_WBoutwherereg<=MEM_WBinlastwherereg;//9
        end
    end
endmodule