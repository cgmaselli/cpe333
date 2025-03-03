`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 01/24/2023 09:30:07 AM
// Module Name: PC_MUX
//////////////////////////////////////////////////////////////////////////////////

module PC_MUX(
    input logic [31:0] PC_OUT_PLUS_FOUR,
    input logic [31:0] JALR,
    input logic [31:0] BRANCH,
    input logic [31:0] JAL,
    input logic [31:0] MTVEC,
    input logic [31:0] MEPC,
    input logic [1:0] PC_SOURCE,
    output logic [31:0] PC_MUX_OUT
    );
    
    //Case statement depending on PC_SOURCE
    always_comb begin
        case(PC_SOURCE)
            2'b00: begin PC_MUX_OUT = PC_OUT_PLUS_FOUR;
            end
            2'b01: begin PC_MUX_OUT = JALR;
            end
            2'b10: begin PC_MUX_OUT = BRANCH;
            end
            2'b11: begin PC_MUX_OUT = JAL;
            end
            default: begin PC_MUX_OUT = 32'h00000000; //Set to 8 to know what the select is, shouldn't happen
            end                                       //shouldn't happen
        endcase
    end
endmodule
