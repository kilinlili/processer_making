
`define  ZERO           1'b0           // Rename to Zero
`define  LOW            1'b0           // Rename to Zero
`define  HIGH           1'b1           // Rename to High

module rf32x32(
		// clock and reset
		clk,reset,//input
		// Inputs
		wr_n,//input regwrite
		rd1_addr, rd2_addr, //wire in 5bits
        wr_addr,//input "out of IDstage" 5bits
		data_in,//input "out of IDstage" 32bits 

		// Outputs
		data1_out, data2_out//wire out 32bits //to forC & forD
		);
   
   parameter data_width      = 32;
   parameter depth           = 32;
   parameter bit_width_depth = 5;  // ceil(log2(depth))
   parameter rst_mode        = 0;  // 0: asynchronously initializes the RAM
                                   // 1: synchronously

   //*** I/O declarations ***//
   input                          clk;       // clock
   input 			  reset;
   input                          wr_n;      // Write enable, active low
   input  [bit_width_depth-1 : 0] rd1_addr;  // Read0 address bus  
   input  [bit_width_depth-1 : 0] rd2_addr;  // Read1 address bus
   input  [bit_width_depth-1 : 0] wr_addr;   // Write address bus
   input       [data_width-1 : 0] data_in;   // Input data bus
   
   output      [data_width-1 : 0] data1_out; // Output data bus for read0
   output      [data_width-1 : 0] data2_out; // Output data bus for read1


   //*** wire declarations ***//
   wire 			  clk_inv;
   wire        [data_width-1 : 0] ram_data1_out;
   wire        [data_width-1 : 0] ram_data2_out;
   

   assign    clk_inv = ~clk;
   assign  data1_out = (|rd1_addr) ? ram_data1_out : {data_width{`ZERO}};
   assign  data2_out = (|rd2_addr) ? ram_data2_out : {data_width{`ZERO}};

   // Instance of DW_ram_2r_w_s_lat
   DW_ram_2r_w_s_dff #(data_width, depth, rst_mode)
      u_DW_ram_2r_w_s_dff(
             .clk(clk_inv), .rst_n(reset),//input "out of IDstage"
             .cs_n(`LOW), .wr_n(wr_n),//cs_n ???? //input "out of IDstage"//Regwrite
             .rd1_addr(rd1_addr), .rd2_addr(rd2_addr),//wire in 
             .wr_addr(wr_addr),//input "out of IDstage"//5bits 
             .data_in(data_in),//input "out of IDstage"//32bits data
             .data_rd1_out(ram_data1_out),//wire out 32bits data
             .data_rd2_out(ram_data2_out)//wire out 32bits data
             //input:6?(3)output:0,
             //wire in 2,wire out 2
      );

endmodule // rf32x32


////////////////////////////////////////////////////////////////////////////////
//
//       This confidential and proprietary software may be used only
//     as authorized by a licensing agreement from Synopsys Inc.
//     In the event of publication, the following notice is applicable:
//
//                    (C) COPYRIGHT 1999 - 2013 SYNOPSYS INC.
//                           ALL RIGHTS RESERVED
//
//       The entire notice above must be reproduced on all authorized
//     copies.
//
// AUTHOR:    Jay Zhu	Sept 22, 1999
//
// VERSION:   Simulation Architecture
//
// DesignWare_version: e321906e
// DesignWare_release: H-2013.03-DWBB_201303.4
//
////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------
// ABSTRACT:  Synch Write, Asynch Dual Read RAM (Flip-Flop Based)
//            (flip flop memory array)
//            legal range:  depth        [ 2 to 2048 ]
//            legal range:  data_width   [ 1 to 1024 ]
//            Input data: data_in[data_width-1:0]
//            Output data from read1: data_rd1_out[data_width-1:0]
//            Output data from read2: data_rd2_out[data_width-1:0]
//            Read1 Address: rd1_addr[addr_width-1:0]
//            Read2 Address: rd2_addr[addr_width-1:0]
//            Write Address: wr_addr[addr_width-1:0]
//            write enable (active low): wr_n
//            chip select (active low): cs_n
//            reset (active low): rst_n
//            clock:clk
//
//	MODIFIED:
//		092299	Jay Zhu		Rewrote for STAR91151
//              10/18/00  RPH       Rewrote accoding to new guidelines 
//                                  STAR 111067   
//              05/25/01  RJK       Rewritten again
//              2/18/09   RJK       Corrected default value for rst_mode
//				    STAR 9000294457
//----------------------------------------------------------------------


module DW_ram_2r_w_s_dff (clk, rst_n, cs_n, wr_n, rd1_addr, rd2_addr, 
			  wr_addr, data_in, data_rd1_out, data_rd2_out);

   parameter data_width = 4;
   parameter depth = 8;
   parameter rst_mode = 1;

`define DW_addr_width ((depth>256)?((depth>4096)?((depth>16384)?((depth>32768)?16:15):((depth>8192)?14:13)):((depth>1024)?((depth>2048)?12:11):((depth>512)?10:9))):((depth>16)?((depth>64)?((depth>128)?8:7):((depth>32)?6:5)):((depth>4)?((depth>8)?4:3):((depth>2)?2:1))))

   input [data_width-1:0] data_in;
   input [`DW_addr_width-1:0] rd1_addr;
   input [`DW_addr_width-1:0] rd2_addr;
   input [`DW_addr_width-1:0] wr_addr;
   input 		      wr_n;
   input 		   rst_n;
   input 		   cs_n;
   input 		   clk;

   output [data_width-1:0] data_rd1_out;
   output [data_width-1:0] data_rd2_out;

// synopsys translate_off
   wire [data_width-1:0]   data_in;
   reg [depth*data_width-1:0]    next_mem;
   reg [depth*data_width-1:0]    mem;
   wire [depth*data_width-1:0]   mem_mux1;
   wire [depth*data_width-1:0]   mem_mux2;
   
   wire 		   a_rst_n;
   

   
  
  initial begin : parameter_check
    integer param_err_flg;

    param_err_flg = 0;
    
	    
  
    if ( (data_width < 1) || (data_width > 2048) ) begin
      param_err_flg = 1;
      $display(
	"ERROR: %m :\n  Invalid value (%d) for parameter data_width (legal range: 1 to 2048)",
	data_width );
    end
  
    if ( (depth < 2) || (depth > 1024 ) ) begin
      param_err_flg = 1;
      $display(
	"ERROR: %m :\n  Invalid value (%d) for parameter depth (legal range: 2 to 1024 )",
	depth );
    end
  
    if ( (rst_mode < 0) || (rst_mode > 1 ) ) begin
      param_err_flg = 1;
      $display(
	"ERROR: %m :\n  Invalid value (%d) for parameter rst_mode (legal range: 0 to 1 )",
	rst_mode );
    end

  
    if ( param_err_flg == 1) begin
      $display(
        "%m :\n  Simulation aborted due to invalid parameter value(s)");
      $finish;
    end

  end // parameter_check
   
   assign mem_mux1 = mem >> (rd1_addr * data_width);

   assign data_rd1_out = ((rd1_addr ^ rd1_addr) !== {`DW_addr_width{1'b0}})? {data_width{1'bx}} : (
				(rd1_addr >= depth)? {data_width{1'b0}} :
				   mem_mux1[data_width-1 : 0] );
   
   assign mem_mux2 = mem >> (rd2_addr * data_width);

   assign data_rd2_out = ((rd2_addr ^ rd2_addr) !== {`DW_addr_width{1'b0}})? {data_width{1'bx}} : (
				(rd2_addr >= depth)? {data_width{1'b0}} :
				   mem_mux2[data_width-1 : 0] );
   
   assign a_rst_n = (rst_mode == 0)? rst_n : 1'b1;


  
   always @ (posedge clk or negedge a_rst_n) begin : registers
      integer i, j;
      
   
      next_mem = mem;

      if ((cs_n | wr_n) !== 1'b1) begin
      
	 if ((wr_addr ^ wr_addr) !== {`DW_addr_width{1'b0}}) begin
	    next_mem = {depth*data_width{1'bx}};	

	 end else begin
         
	    if ((wr_addr < depth) && ((wr_n | cs_n) !== 1'b1)) begin
	       for (i=0 ; i < data_width ; i=i+1) begin
		  j = wr_addr*data_width + i;
		  next_mem[j] = ((wr_n | cs_n) == 1'b0)? data_in[i] | 1'b0
					: mem[j];
	       end // for
	    end // if
	 end // if-else
      end // if   
   
   
      if (rst_n === 1'b0) begin
         mem <= {depth*data_width{1'b0}};
      end else begin
         if ( rst_n === 1'b1) begin
	    mem <= next_mem;
	 end else begin
	    mem <= {depth*data_width{1'bX}};
	 end
      end
   end // registers
   
    
  always @ (clk) begin : clk_monitor 
    if ( (clk !== 1'b0) && (clk !== 1'b1) && ($time > 0) )
      $display( "WARNING: %m :\n  at time = %t, detected unknown value, %b, on clk input.",
                $time, clk );
    end // clk_monitor 

// synopsys translate_on

`undef DW_addr_width
endmodule // DW_ram_2r_w_s_dff
