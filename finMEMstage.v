module mem1(
    MEM_inMEMWRITE,MEM_inMEMREAD,
    //------------------------------------------------------------------------------------------
    MEM_outMREQ
);
    input MEM_inMEMWRITE,MEM_inMEMREAD;
    output MEM_outMREQ;

    logicsum userlogicsum(
        .dataA(MEM_inMEMWRITE),.dataB(MEM_inMEMREAD),
        .outC(MEM_outMREQ)
    );

endmodule 

/*
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


    logicsum userlogicsum(
        .dataA(),.dataB(),
        .outC()
    );

endmodule 
*/