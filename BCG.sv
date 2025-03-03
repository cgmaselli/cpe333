`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly San Luis Obispo
// Engineer: Diego Curiel
// Create Date: 02/09/2023 11:30:51 AM
// Module Name: BCG
//////////////////////////////////////////////////////////////////////////////////

module BCG(
    input logic [31:0] RS1,
    input logic [31:0] RS2,

    input logic IR_30,
    input logic [6:0] IR_OPCODE,
    input logic [2:0] IR_FUNCT,
    
//    output logic BR_EQ,
//    output logic BR_LT,
//    output logic BR_LTU,
    output logic [1:0] PC_SOURCE
    );
    
    logic BR_LT, BR_EQ, BR_LTU;
    
    //Assign outputs using conditional logic operators.
    assign BR_LT = $signed(RS1) < $signed(RS2);
    assign BR_LTU = RS1 < RS2;
    assign BR_EQ = RS1 == RS2;
    
    always_comb begin
        //Instantiate all outputs to 0 so as to avoid
        //unwanted leftovers from previous operations
        //and maintain direct control of outputs through
        //case statement below
        PC_SOURCE = 2'b00;
        
        //Case statement depending on the opcode for the 
        //instruction, or the last seven bits of each instruction
        case (IR_OPCODE)
            7'b1101111: begin // JAL
                PC_SOURCE = 2'b11;
            end
            
            7'b1100111: begin // JALR
                PC_SOURCE = 2'b01;
            end

            7'b1100011: begin // B-Type
                //nested case statement dependent on the
                //function three bits.
                //Because there are six real branch instructions, there
                //are six pairs of if-else statements in each of six cases
                //for the branch instructions.
                case(IR_FUNCT)
                    3'b000: begin
                        if (BR_EQ == 1'b1)
                            PC_SOURCE = 2'b10;
                        else
                            PC_SOURCE = 2'b00; 
                    end
                    3'b001: begin 
                        if (BR_EQ == 1'b0)
                            PC_SOURCE = 2'b10;
                        else
                            PC_SOURCE = 2'b00; 
                    end
                    3'b100: begin 
                        if (BR_LT == 1'b1)
                            PC_SOURCE = 2'b10;
                        else
                            PC_SOURCE = 2'b00;
                    end
                    3'b101: begin 
                        if (BR_LT == 1'b0)
                            PC_SOURCE = 2'b10;
                        else
                            PC_SOURCE = 2'b00;
                    end
                    3'b110: begin 
                        if (BR_LTU == 1'b1)
                            PC_SOURCE = 2'b10;
                        else
                            PC_SOURCE = 2'b00;
                    end
                    3'b111: begin 
                        if (BR_LTU == 1'b0)
                            PC_SOURCE = 2'b10;
                        else
                            PC_SOURCE = 2'b00;
                    end
                endcase
            end
            
            default: begin end
        endcase
    end

endmodule
