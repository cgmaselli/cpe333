`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/26/2025 08:36:16 PM
// Design Name: 
// Module Name: HazardUnit
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


module HazardUnit(
        input logic [6:0] opcode,
        input logic [4:0] de_adr1,  // de
        input logic [4:0] de_adr2,  // de
        input logic [4:0] de_ex_adr1,   // ex
        input logic [4:0] de_ex_adr2,   // ex
        input logic [4:0] de_ex_rd, // ex rd
        input logic [4:0] ex_mem_rd,    // mem rd, destination register
        input logic [4:0] mem_wb_rd,    // wb rd
        input logic [1:0] pc_source,
        input logic ex_mem_regWrite,    // mem rw
        input logic mem_wb_regWrite,    // wb rw
        input logic de_ex_memread,
        input logic mem_wb_memread,
        input logic ex_mem_memwrite,
        input logic de_rs1_used,
        input logic de_rs2_used,
        input logic de_ex_rs1_used, // ex
        input logic de_ex_rs2_used, // ex
        
        output logic [1:0] fsel1,
        output logic [1:0] fsel2,
        output logic off_sel,
        output logic store_load_haz,
        output logic load_use_haz,
        output logic control_haz,
        output logic flush
    );
    
    always_comb begin
        fsel1 = 2'b00;
        fsel2 = 2'b00;
        off_sel = 1'b0;
        store_load_haz = 1'b0;
        load_use_haz = 1'b0;
        control_haz = 1'b0;
        flush = 1'b0;
        
        //Selects for forwarding muxes, data hazards?
        // name of destination is equal to name of source register on prev instruction then will set mux to correct value, in this case it is 1
        // can have this on mem stage? want two instructions, have about 4 conditions
        // result will be ex_mem_rd == de_ex_adr1 && de_ex_rs1_used && ex_mem_regWrite
        // check conditions, what is true/false?
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!!!!!!!! check the book for branch conditions please god !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        
        // ex hazard
        if (ex_mem_regWrite && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_adr1)) begin
            fsel1 = 2'b10;  // change select value to whatever matches our mux, hers is diff from the book?
        end
        else if (mem_wb_regWrite && (mem_wb_rd != 0) && !(ex_mem_regWrite && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_adr1)) && (mem_wb_rd == de_ex_adr1)) begin
            fsel1 = 2'b01;
        end
        else begin
            fsel1 = 2'b00;
        end
        
        if (ex_mem_regWrite && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_adr2)) begin
            fsel2 = 2'b10;
        end
        else if (mem_wb_regWrite && (mem_wb_rd != 0) && !(ex_mem_regWrite && (ex_mem_rd != 0) && (ex_mem_rd == de_ex_adr2)) && (mem_wb_rd == de_ex_adr2)) begin
            fsel2 = 2'b01;
        end
        else begin
            fsel2 = 2'b00;
        end
        
        // Store after load hazard
        // mem read is only on for load instructions
        // mem write is only on for store instructions
        // do we need to check if the destination for load is x0?
        // do we need to check if the destination address for store is x0?
        if (mem_wb_memread && ex_mem_memwrite && !(mem_wb_rd != 0)) begin
            store_load_haz = 1'b1;
        end
        
        if (opcode == 7'b0000011 || opcode == 7'b0100011) begin
            off_sel = 1'b1;
        end
        else
        begin
            off_sel = 1'b0;
        end
            
        //Load-use data hazard
        // load instruction if data mem read enable is set to true bc load only instruction to set that to true
        // also have to check names of source and destination registers and if it is true, in this case set flag ld use hazard = 1
        // which is going to trigger the stall, only stall in 1 case (read above write or ???)
        // set hazard to 1
        // need to handle bubble inserted later on, when stalling don't want pc to write?
        // check opcode or the read enable? which is right?
        
//        if (de_ex_memread && ((de_ex_rd == de_adr1) || (de_ex_rd == de_adr2))) begin
//            load_use_haz = 1'b1;
//        end
        if ((opcode == 7'b0000011) && ((de_adr1 == de_ex_rd && de_rs1_used) || (de_adr2 == de_ex_rd && de_rs2_used)))
        begin
            load_use_haz = 1'b1;
        end
        else
        begin
            load_use_haz = 1'b0;
        end
            
        //Control hazards - jal, jalr, branch
        // if pc source is (mux that determines what needs to be put into pc so it can be pc + 4, label, etc.
        // if pc source is not regular one (00, pc + 4), there is a control hazard and MAYBE need to flush
        // branch and then flush because whatever i load is going to be wrong, these are just wires set to 1
        // have to reset the flags so when pc stops executing the branch
        if (pc_source != 2'b00) begin
            control_haz = 1'b1;
            flush = 1'b1;
        end
        else begin
            control_haz = 1'b0;
            flush = 1'b0;
        end
    end
endmodule
// in cache file at bottom, included tags at index and ?C? tag, it never updates any of the tags right now, inside always block
