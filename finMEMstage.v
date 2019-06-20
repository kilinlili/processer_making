module mem1(
    MEM_inLOADDATA,
    //------------------------------------------------------------------------------------------
    MEM_outSIZE,
    MEM_outMEMWRITE,MEM_outMEMREAD,
    MEM_outADDRESS,MEM_outWRITEDATA,

);
    input[31:0] MEM_inLOADDATA;
    //-------------------------------------------------------------------------------------------
    output [1:0] MEM_outSIZE;
    output MEM_outMEMWRITE,MEM_outMEMREAD;
    output [31:0] MEM_outADDRESS,MEM_outWRITEDATA;

endmodule 