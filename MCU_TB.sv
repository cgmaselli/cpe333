`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/24/2024 04:45:15 PM
// Design Name: 
// Module Name: MCU_TB
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module MCU_TB();

    logic clk, intr, rst, wr;
    logic [31:0] in, iobus_out, iobus_addr;
    
    OTTER_MCU UUT( .CLK(clk), .IOBUS_IN(in), .INTR(intr), .RESET(rst), 
              .IOBUS_OUT(iobus_out), .IOBUS_ADDR(iobus_addr), .IOBUS_WR(wr) );

    initial begin
    clk = 0;
    rst = 1'b1;
    end
    
    always begin
    #10 clk = ~clk; end  

    always begin
    #60 rst = 0;
     in = 32'h2;
    end


endmodule
