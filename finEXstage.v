module ex1(
    EX_inIDEXREGISTERRT,EX_inIDEXREGISTERRD,
    EX_inREGDEST,
    //mux5.v //32register sitei
    EX_inIDEXREGISTERRS,//&EX_inIDEXREGISTERRT
    EX_inEXMEMREGISTERRDRT,EX_inMEMWBREGISTERRDRT,
    EX_inEXMEMREGWRITE,EX_inMEMWBREGWRITE,
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    EX_injalsig,EX_injalrsig,EX_inbalal,
    //allsum.v
    EX_infromRs,EX_infromRt,EX_infromMEMWB,EX_infromEXMEM,
    //forA,forB
    EX_infromshamt,
    EX_inshamtsignal,
    //shamt
    EX_iniformat,
    EX_inALUSRC,
    //EX_inalusrcmux 
    EX_infromALUctl,
    EX_infromIALUctl,
    //----------------------------------------------------------------------------------------------
    EX_outanswer,
    EX_outtoANDLINK,//-------------->to pipeline
    EX_outtopipereg5,
    EX_outfboutpipe
    //ALU
);
    input [4:0] EX_inIDEXREGISTERRT,EX_inIDEXREGISTERRD;
    input EX_inREGDEST;
    //mux5.v //32register sitei
    input [4:0] EX_inIDEXREGISTERRS;//&EX_inIDEXREGISTERRT
    input [4:0] EX_inEXMEMREGISTERRDRT,EX_inMEMWBREGISTERRDRT;
    input EX_inEXMEMREGWRITE,EX_inMEMWBREGWRITE;
    //temp_FORWARDING_UNIT1.v//FORWARDING_UNIT1.v
    input EX_injalsig,EX_injalrsig,EX_inbalal;
    //allsum.v
    input [31:0]EX_infromRs,EX_infromRt,EX_infromMEMWB,EX_infromEXMEM;
    //forA,forB
    input [4:0] EX_infromshamt;
    input EX_inshamtsignal;
    //shamt
    input [31:0] EX_iniformat;
    input EX_inALUSRC;
    //EX_inalusrcmux 
    input [3:0]EX_infromALUctl;
    input [3:0]EX_infromIALUctl;
    //-------------------------------------------------------------------------------

    output [31:0]EX_outanswer;
    output EX_outtoANDLINK;//-------------->to pipeline
    output [4:0] EX_outtopipereg5;
    output [31:0]EX_outfboutpipe;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*wire call*/
    wire [4:0] rtrd;//muxfive
    wire [1:0] forA,forB;//forwarding1
    wire anlink;
    wire muxsig;//allsum
    //wire [4:0]goline; //-> pipeline//direct!
    /*wire call*/

    /*wire call*/
    wire [31:0] faout,fbout;//forA out & forB out //////////////////////////////
    wire [31:0] toALUA; //shamtmux.v out
    wire [31:0] toALUB; //EX_inALUsrcmux out 
    //wire [31:0] EX_outanswerwire;//direct!
    /*wire call*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*fo to pipeline */
    //assign EX_outanswer = EX_outanswerwire;//direct!
    //assign alink  = anlink;
    //assign Register = goline;
    assign EX_outfboutpipe = fbout;
/////////////////////////////////////////////////

    muxfive muxreg5(
        .data1(EX_inIDEXREGISTERRT),
        .data2(EX_inIDEXREGISTERRD),
        .signal(EX_inREGDEST),
        .out(rtrd)
    );
    forwarding1 forwa1(
        .ID_EX_RegisterRs(EX_inIDEXREGISTERRS),.ID_EX_RegisterRt(EX_inIDEXREGISTERRT),
        .EX_MEM_RegisterR(EX_inEXMEMREGISTERRDRT),.MEM_WB_RegisterR(EX_inMEMWBREGISTERRDRT),
        .EX_MEM_RegWrite(EX_inEXMEMREGWRITE),.MEM_WB_RegWrite(EX_inMEMWBREGWRITE),
        .ForwardA(forA),.ForwardB(forB)//->forA,forB 2bit
    );
    allsum alls(
        .EX_injalsig(EX_injalsig),.EX_injalrsig(EX_injalrsig),.EX_inbalal(EX_inbalal),
        .andlink(EX_outtoANDLINK),.regictl(muxsig)
        //alink goes pipeline
        //muxsig is mux signal
    );
    mux31 mux31(
        .regi(rtrd),.signal(muxsig),
        .out(EX_outtopipereg5)//------------>to pipeline 
    );

    fmux forwardingA(
        .data1(EX_infromRs),.data2(EX_infromEXMEM),.data3(EX_infromMEMWB),.signal(forA),
        .out(faout)//->shamtmux.v 
    );
    fmux forwardingB(
        .data1(EX_infromRt),.data2(EX_infromEXMEM),.data3(EX_infromMEMWB),.signal(forB),
        .out(fbout)//=>EX_inALUSrcmux & => to pipeline
    );
    shmux shamtjudge(
        .shamtsig(EX_inshamtsignal),.shamt(EX_infromshamt),.fromR(faout),
        .out(toALUA)
    );
    //EX_inALUsrc
    mux arusrcmux(
        .data1(EX_iniformat),.data2(fbout),.signal(EX_inALUSRC),
        .out(toALUB)
    );
    mainALU ALU(
        .ALUctl(EX_infromALUctl),.IALUctl(EX_infromIALUctl),
        .A(toALUA),.B(toALUB),
        .ALUOut(EX_outanswer)//to pipeline 
    );

    /*
    to pipeline !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    EX_outanswer
    alink
    muxsig
    fbout
    */




endmodule