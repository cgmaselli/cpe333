`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
    logic [31:0] next_pc;
} instr_t;

typedef struct packed {
    logic [31:0] U;
    logic [31:0] S;
    logic [31:0] I;
    logic [31:0] J;
    logic [31:0] B;
} immed_t;

module OTTER_MCU(input CLK,
                input INTR, // use later
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    logic [6:0] opcode;
    logic [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc, A, B,
        I_immed, S_immed, U_immed, B_immed, J_immed, aluBin, aluAin, aluResult,
        rfIn, csr_reg, mem_data, rs1_out, rs2_out;
    
    logic [31:0] IR;
    logic memRead1,memRead2;
    
    logic pcWrite, regWrite, memWrite, op1_sel, mem_op, IorD, pcWriteCond, memRead;
    logic [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    logic [3:0] alu_fun;
    logic opA_sel;
    
    immed_t DE_IMM, EX_IMM, MEM_IMM, WB_IMM;
    
//    logic br_lt, br_eq, br_ltu;
    // instr_t decode, execute;
              
//==== Instruction Fetch ===========================================
      assign pcWrite = 1'b1; 	//Hardwired high, assuming no hazards
      assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
     // Logic signals    
     logic [31:0] if_de_pc;
     logic [31:0] if_de_next_pc;
     logic [31:0] ex_mem_rs2;
     logic [31:0] ex_mem_aluRes;
     logic [1:0] ex_mem_size;
     logic ex_mem_sign;
     
     // Hazards - added from lab 3 video
   //  logic [31:0] jalr_if, jal_if, branch_if, pc_if, pc_inc_if; // might not need, might want to rename, might already have, who knows
    // logic err;
    // logic ld_use_hz, flush, hold_flush;
  
     
     PC OTTER_PC(.CLK(CLK), .RST(RESET), .PC_WRITE(pcWrite), .PC_SOURCE(pc_sel),
        .JALR(jalr_pc), .JAL(jump_pc), .BRANCH(branch_pc), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc), .PC_OUT_INC(next_pc));
     
     
    OTTER_mem_byte OTTER_mem_byte(.MEM_CLK(CLK), .MEM_ADDR1(pc), .MEM_ADDR2(ex_mem_aluRes),
        .MEM_DIN2(ex_mem_rs2), .MEM_WRITE2(ex_mem_inst.memWrite), .MEM_READ1(memRead1),
        .MEM_READ2(ex_mem_inst.memRead2), .ERR(err), .MEM_DOUT1(IR), .MEM_DOUT2(mem_data), .IO_IN(IOBUS_IN),
        .IO_WR(IOBUS_WR), .MEM_SIZE(ex_mem_size), .MEM_SIGN(ex_mem_sign));
        
     
     always_ff @(posedge CLK) begin     // pipeline reg
        //    if(!ld_use_hz) begin
            // put in conditions, fix code, throw these values somewhere
          //  end
                if_de_pc <= pc;
                if_de_next_pc <= next_pc;
     end
     
   //  assign pcWrite = !ld_use_hz;
    // assign memRead1 = !ld_use_hz;
     
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA; // output of mux
    logic [31:0] de_ex_opB; // output of mux
    logic [31:0] de_ex_rs1;
    logic [31:0] de_ex_rs2; // used again in mem
    logic [31:0] de_ex_IR;
    logic [31:0] de_ex_data;
    logic [24:0] de_ex_igin;
    assign opcode = IR[6:0];
    instr_t de_ex_inst, de_inst;
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst.pc = if_de_pc;
    assign de_inst.next_pc = if_de_next_pc;
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.mem_type=IR[14:12];
    assign de_ex_igin=IR[31:7];
    assign de_inst.opcode=OPCODE;
   
    assign de_inst.rs1_used=    de_inst.rs1_addr != 0    // had set to rs1_out != 0, might change back
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
    
    // also assign rs2 and rd used
 //   assign de_inst.rs2_used = ....
    
 //   assign de_inst.rd_used = ....

                                
    CU_DCDR OTTER_DCDR(.IR_30(IR[30]), .IR_OPCODE(de_inst.opcode), .IR_FUNCT(de_inst.mem_type), .REG_WRITE(de_inst.regWrite),
        .MEM_WE2(de_inst.memWrite), .MEM_RDEN2(de_inst.memRead2), .ALU_FUN(de_inst.alu_fun), .ALU_SRCA(opA_sel),
        .ALU_SRCB(opB_sel), .RF_WR_SEL(de_inst.rf_wr_sel));
    
    // reg file
    REG_FILE REG_FILE(.CLK(CLK), .EN(mem_wb_inst.regWrite), .ADR1(de_inst.rs1_addr), .ADR2(de_inst.rs2_addr), .WA(mem_wb_inst.rd_addr), .WD(rfIn), .RS1(rs1_out), .RS2(rs2_out));
    
    // imm gen
    ImmediateGenerator ImmediateGenerator(.IR(de_ex_igin), .U_TYPE(U_immed), .I_TYPE(I_immed), .S_TYPE(S_immed), .B_TYPE(B_immed), .J_TYPE(J_immed));
    assign DE_IMM.J = J_immed;
    assign DE_IMM.B = B_immed;
    assign DE_IMM.I = I_immed;
    assign DE_IMM.S = S_immed;
    assign DE_IMM.U = U_immed;
    
     TwoMux TwoMux(.SEL(opA_sel), .ZERO(rs1_out), .ONE(U_immed), .OUT(aluAin));
    // TwoMux TwoMux(.ALU_SRC_A(opA_sel), .RS1(frs1_ex), .U_TYPE(U_immed), .SRC_A(aluAin));
     FourMux FourMux(.SEL(opB_sel), .ZERO(rs2_out), .ONE(I_immed), .TWO(S_immed), .THREE(de_inst.pc), .OUT(aluBin));
   // FourMux FourMux(.SEL(opB_sel), .ZERO(frs2_ex), .ONE(I_immed), .TWO(S_immed), .THREE(de_inst.pc), .OUT(aluBin));
    
    always_ff@(posedge CLK) begin   // pipeline reg
        de_ex_inst <= de_inst;
        EX_IMM <= DE_IMM;
        de_ex_opA <= aluAin;
        de_ex_opB <= aluBin;
        de_ex_rs1 <= rs1_out;
        de_ex_rs2 <= rs2_out;
        de_ex_IR <= IR;
       // if (flush ...) begin    // bubbles for hazard handling
       //     // check register that this is in
       //     de_ex_inst.regWrite <= 1'b0;
       //    de_ex_inst.memWrite <= 1'b0;
       //     de_ex_IR <= 32'd0;  // ?
        // end
    end
		

//==== Hazard Handling, Forwarding (where are we even putting this) ===

    logic [1:0] fsel1, fsel2;
    logic [31:0] frs1_ex, frs2_ex;
    logic [6:0] ex_load_op;
    logic cntrl_haz;
    assign ex_load_op = de_ex_IR[6:0];
    
//    HazardUnit HazardUnit(.opcode(ex_load_op), .de_adr1(de_inst.rs1_addr), .de_adr2(de_inst.rs2_addr), .ex_adr1(de_ex_inst.rs1_addr), .ex_adr2(de_ex_inst.rs2_addr),
//        .ex_rd(de_ex_inst.rd_addr), .mem_rd(ex_mem_inst.rd_addr), .wb_rd(mem_wb_inst.rd_addr), .pc_source(pcSource), .mem_regWrite(ex_mem_inst.regWrite),
//        .wb_regWrite(mem_wb_inst.regWrite), .de_rs1_used(de_inst.rs1_used), .de_rs2_used(de_inst.rs2_used), .ex_rs1_used(de_ex_inst.rs1_used), .ex_rs2_used(de_ex_inst.rs2_used),
//        .fsel1(fsel1), .fsel2(fsel2), .load_use_haz(ld_use_hz), .control_haz(cntrl_haz), .flush(flush));
        
    //  assign fsel1 = 1'b0; this was already commented out, i don't really know why or what this is for yet
    // i don't know where half of these values are being instantiated rn, check what she has
    // FourMux FRS1(.SEL(fsel1), .ZERO(de_ex_rs1), .ONE(mem_wd), .TWO(rfIn), .THREE(32'h00000000), .OUT(frs1_ex));
    
    // FourMux FRS2(.SEL(fsel2), .ZERO(de_ex_rs2), .ONE(mem_wd), .TWO(rfIn), .THREE(32'h00000000), .OUT(frs2_ex));

//==== Execute ========================================================

     instr_t ex_mem_inst;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     logic [31:0] ex_mem_rs2;
     
     assign opA_forwarded = de_ex_opA;
     assign opB_forwarded = de_ex_opB;
     assign ex_mem_size = ex_mem_inst.mem_type[1:0];
     assign ex_mem_sign = ex_mem_inst.mem_type[2];
     
    BAG OTTER_BAG(.RS1(de_ex_rs1), .I_TYPE(EX_IMM.I), .J_TYPE(EX_IMM.J), .B_TYPE(EX_IMM.B), .FROM_PC(de_ex_inst.pc),
        .JAL(jump_pc), .JALR(jalr_pc), .BRANCH(branch_pc));

 //    BAG OTTER_BAG(.RS1(frs1_ex), .I_TYPE(EX_IMM.I), .J_TYPE(EX_IMM.J), .B_TYPE(EX_IMM.B), .FROM_PC(de_ex_inst.pc),
 //        .JAL(jump_pc), .JALR(jalr_pc), .BRANCH(branch_pc));
    // are we going to put in new, forwarded pc values? or does it not matter? check her naming conventions
    
    
     BCG OTTER_BCG(.RS1(de_ex_rs1), .RS2(de_ex_rs2), .IR_30(de_ex_IR[30]), .IR_OPCODE(de_ex_inst.opcode), .IR_FUNCT(de_ex_inst.mem_type),
         .PC_SOURCE(pc_sel));
        
    // BCG OTTER_BCG(.RS1(frs1_ex), .RS2(frs2_ex), .IR_30(de_ex_IR[30]), .IR_OPCODE(de_ex_inst.opcode), .IR_FUNCT(de_ex_inst.mem_type),
    //    .PC_SOURCE(pc_sel));
        // pc_sel or pcSource????????????
     
     // Creates a RISC-V ALU
     // changing values, need to instantiate w/ potentially forwarded values
    ALU OTTER_ALU(.SRC_A(opA_forwarded), .SRC_B(opB_forwarded), .ALU_FUN(de_ex_inst.alu_fun), .RESULT(aluResult));
    
    always_ff@(posedge CLK) begin   // pipeline reg
        ex_mem_inst <= de_ex_inst;
        ex_mem_aluRes <= aluResult;
        ex_mem_rs2 <= de_ex_rs2;
        MEM_IMM <= EX_IMM;
    end


//==== Memory ======================================================
     
    instr_t mem_wb_inst;
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    logic [31:0] mem_wb_aluRes;
    // assign de_ex_inst.ir = 32'b0;
    
    // wtf is mem_wd!
     
    always_ff@(posedge CLK) begin   // pipeline reg
        mem_wb_inst <= ex_mem_inst;
        mem_wb_aluRes <= ex_mem_aluRes;
        WB_IMM <= MEM_IMM;
    end
 
     
//==== Write Back ==================================================
    
    FourMux OTTER_REG_MUX(.SEL(mem_wb_inst.rf_wr_sel), .ZERO(mem_wb_inst.next_pc), .ONE(32'b0),
        .TWO(mem_data), .THREE(mem_wb_aluRes), .OUT(rfIn)); 

            
endmodule
