`timescale 1ns/100ps  // 1 ns time unit, 100 ps resolution
`default_nettype none // Makes it easier to detect typos !

module test_sdram;
    reg clk;
    always #25 clk = !clk;

    reg resetq = 0;

   /***************************************************************************/
   // SD-RAM-Controller
   /***************************************************************************/
  wire uart_loop;
  wire [3:0] spi;
  assign spi[0] = 1;
  core_top dut(
    .clk_74a(clk),
    .reset_n(~resetq),

    .dram_clk(sdram_clk),
    .dram_cke(sdram_cke),
    .dram_we_n(sdram_wen),
    .dram_ras_n(sdram_rasn),
    .dram_cas_n(sdram_casn),
    .dram_a(sdram_a),
    .dram_ba(sdram_ba),
    .dram_dqm(sdram_dqm),
    .dram_dq(sdram_d)
);


   /***************************************************************************/
   // 64 MB SD-RAM
   /***************************************************************************/

  wire  sdram_csn;       // chip select
  wire  sdram_clk;       // clock to SDRAM
  wire  sdram_cke;       // clock enable to SDRAM
  wire  sdram_rasn;      // SDRAM RAS
  wire  sdram_casn;      // SDRAM CAS
  wire  sdram_wen;       // SDRAM write-enable
  wire [12:0] sdram_a;   // SDRAM address bus
  wire  [1:0] sdram_ba;  // SDRAM bank-address
  wire  [1:0] sdram_dqm; // byte select
  wire [15:0] sdram_d;

  mt48lc16m16a2 memory(
   .Dq(sdram_d),
   .Addr(sdram_a),
   .Ba(sdram_ba),
   .Clk(sdram_clk),
   .Cke(sdram_cke),
   .Cs_n(1'b0),
   .Ras_n(sdram_rasn),
   .Cas_n(sdram_casn),
   .We_n(sdram_wen),
   .Dqm(sdram_dqm)
   );

   /***************************************************************************/
   // Test sequence
   /***************************************************************************/

   integer i;
   initial begin
     $dumpfile("sdram.vcd");    // create a VCD waveform dump
     $dumpvars(0, test_sdram);  // dump variable changes in the testbench
                                // and all modules under it

     clk = 0;
     resetq = 1;
     #1000;
     resetq = 0;

     #20000000;

     $finish();
   end
endmodule
