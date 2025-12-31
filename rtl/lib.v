// Prpgram Counter //

`timescale 1ns/1ps
module ProgramCounter(
    input        clk,
    input        rst,
    input        pc_write,   // from hazard unit
    input [31:0] pc_next,
    output reg [31:0] pc
);
    always @(posedge clk) begin
        if (rst)
            pc <= 32'b0;
        else if (pc_write)
            pc <= pc_next;   
        // else: hold PC (stall)
    end
endmodule

// Instruction Register //

`timescale 1ns/1ps
module Instr_mem(
    input  [29:0] addr,
    output [31:0] instr
);
    reg [31:0] mem [0:1023];

    assign instr = mem[addr];

endmodule

// General Purpose Register //

`timescale 1ns/1ps
module Regfile(
    input clk,
    input rst,              
    input we,
    input [4:0] rs1, rs2, rd,
    input [31:0] rd_data,
    output [31:0] rs1_data, rs2_data
);
    reg [31:0] regs [0:31];
    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i+1)
                regs[i] <= 32'b0;
        end else begin
            if (we && rd != 5'b0)
                regs[rd] <= rd_data;
        end
    end

    assign rs1_data = (rs1 == 5'd0) ? 32'b0 :
                  (we && (rd == rs1)) ? rd_data : regs[rs1];

    assign rs2_data = (rs2 == 5'd0) ? 32'b0 :
                  (we && (rd == rs2)) ? rd_data : regs[rs2];

endmodule  

// Alu Control Unit //

`timescale 1ns/1ps
module Alu_CtrlUnit(
  input [2:0] funct3,
  input [6:0] funct7,
  input [1:0] alu_op,
  output reg [3:0] alu_ctrl
);
  always @(*) begin
    case(alu_op)
      2'b00: alu_ctrl = 4'b0000;
      2'b01: alu_ctrl = 4'b0001;
      2'b10: begin
        case (funct3)
          3'b000: alu_ctrl = (funct7 == 7'b0100000) ? 4'b0001 : 4'b0000;
          3'b111: alu_ctrl = 4'b0010;
          3'b110: alu_ctrl = 4'b0011;
          3'b100: alu_ctrl = 4'b0100;
          3'b001: alu_ctrl = 4'b0101;
          3'b101: alu_ctrl = (funct7 == 7'b0100000) ? 4'b0111 : 4'b0110;
          3'b010: alu_ctrl = 4'b1000;
          3'b011: alu_ctrl = 4'b1001;
          default: alu_ctrl = 4'b0000;
        endcase
      end
      default: alu_ctrl = 4'b0000;
    endcase
  end
endmodule

// Alu //

`timescale 1ns/1ps
module Alu(
  input [31:0] a, b,
  input [3:0] ctrl,
  output reg [31:0] acc,
  output zero_flag
);

  always @(*) begin
    case(ctrl)
      4'b0000: acc = a + b;              // ADD
      4'b0001: acc = a - b;              // SUB
      4'b0010: acc = a & b;              // AND
      4'b0011: acc = a | b;              // OR
      4'b0100: acc = a ^ b;              // XOR
      4'b0101: acc = a << b[4:0];       // Shift left logical
      4'b0110: acc = a >> b[4:0];       // Shift right logical
      4'b0111: acc = $signed(a) >>> b[4:0]; // Shift right arithmetic
      4'b1000: acc = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;  // Set less than (signed)
      4'b1001: acc = (a < b) ? 32'd1 : 32'd0;                    // Set less than (unsigned)
      default: acc = 32'd0;
    endcase
  end

  assign zero_flag = (acc == 32'd0);

endmodule

// Control Unit //
 
`timescale 1ns/1ps
module Control_unit(
    input  [6:0] opcode,
    input  [2:0] funct3,
    input  [6:0] funct7,

    output reg       reg_write,
    output reg       alu_src,
    output reg       mem_read,
    output reg       mem_write,
    output reg       mem_to_reg,
    output reg       branch,
    output reg       jump,
    output reg       mac_en,
    output reg       alu_en,
    output reg [1:0] alu_op
);

always @(*) begin
    // ================= DEFAULTS =================
    reg_write  = 0;
    alu_src    = 0;
    mem_read   = 0;
    mem_write  = 0;
    mem_to_reg = 0;
    branch     = 0;
    jump       = 0;
    mac_en     = 0;
    alu_en     = 0;
    alu_op     = 2'b00;

    case (opcode)

        // --------------------------------------------------
        // CUSTOM AI MAC (INT8 SIMD MAC + ReLU)
        // --------------------------------------------------
        7'b0001011: begin   // CUSTOM-0
                reg_write = 1;
                mac_en    = 1;
        end

        // --------------------------------------------------
        // R-TYPE ALU (ADD, SUB, AND, OR, etc.)
        // --------------------------------------------------
        7'b0110011: begin
            reg_write = 1;
            alu_en    = 1;
            alu_op    = 2'b10;
        end

        // --------------------------------------------------
        // I-TYPE ALU (ADDI, ANDI, ORI, etc.)
        // --------------------------------------------------
        7'b0010011: begin
            reg_write = 1;
            alu_src   = 1;
            alu_en    = 1;
            alu_op    = 2'b10;
        end

        // --------------------------------------------------
        // LOAD (LB, LH, LW, LBU, LHU)
        // --------------------------------------------------
        7'b0000011: begin
            reg_write  = 1;
            mem_read   = 1;
            mem_to_reg = 1;
            alu_src    = 1;
            alu_en     = 1;
            alu_op     = 2'b00;   // base + offset
        end
        7'b0110111 : begin
            reg_write  = 1;
            alu_src    = 1;
            alu_en     = 1;
        end

        // --------------------------------------------------
        // STORE (SB, SH, SW)
        // --------------------------------------------------
        7'b0100011: begin
            mem_write = 1;
            alu_src   = 1;
            alu_en    = 1;
            alu_op    = 2'b00;   // base + offset
        end

        // --------------------------------------------------
        // BRANCH (BEQ, BNE, BLT, BGE)
        // --------------------------------------------------
        7'b1100011: begin
            branch = 1'b1;  
        end

        // --------------------------------------------------
        // JAL
        // --------------------------------------------------
        7'b1101111: begin
            reg_write = 1;    // write PC+4
            jump      = 1;
        end

        // --------------------------------------------------
        // JALR
        // --------------------------------------------------
        7'b1100111: begin
            reg_write = 1;
            jump      = 1;
            alu_src   = 1;    // rs1 + imm
        end

        default: begin
            // NOP
        end

    endcase
end

endmodule

// Hazard Detection //

`timescale 1ns/1ps
module Hazard_detection_unit(
    input  wire [4:0] ID_rs1,
    input  wire [4:0] ID_rs2,
    input  wire [4:0] EX_rd,
    input  wire       EX_memread,
    output reg        stall,
    output reg        pc_write,
    output reg        ifid_write,
    output reg        idex_flush
);
    always @(*) begin
        // default: no stall
        stall      = 1'b0;
        pc_write   = 1'b1;
        ifid_write = 1'b1;
        idex_flush = 1'b0;

        // Load-use hazard:
        // EX is LW and ID uses same rd as rs1/rs2
        if (EX_memread && ((EX_rd == ID_rs1) || (EX_rd == ID_rs2)) && (EX_rd != 5'd0)) begin
            stall      = 1'b1;
            pc_write   = 1'b0;   // freeze PC
            ifid_write = 1'b0;   // freeze IF/ID
            idex_flush = 1'b1;   // insert bubble into ID/EX
        end
    end
endmodule

// Forward Unit //

`timescale 1ns/1ps
module forwarding_unit(
    input [4:0] EX_rs1,
    input [4:0] EX_rs2,
    input [4:0] MEM_rd,
    input [4:0] WB_rd,
    input       MEM_regwrite,
    input       WB_regwrite,
    output reg [1:0] forwardA,
    output reg [1:0] forwardB
);

always @(*) begin
    forwardA = 2'b00;
    forwardB = 2'b00;

    // --- EX hazard: MEM stage ---
    if (MEM_regwrite && (MEM_rd != 0) && (MEM_rd == EX_rs1))
        forwardA = 2'b10;
    else if (WB_regwrite && (WB_rd != 0) && (WB_rd == EX_rs1))
        forwardA = 2'b01;

    if (MEM_regwrite && (MEM_rd != 0) && (MEM_rd == EX_rs2))
        forwardB = 2'b10;
    else if (WB_regwrite && (WB_rd != 0) && (WB_rd == EX_rs2))
        forwardB = 2'b01;
end

endmodule

// Data Memory //

`timescale 1ns/1ps
module Data_mem(
    input clk,
    input mem_read,
    input mem_write,
    input [31:0] addr,
    input [31:0] wdata,
    output reg [31:0] rdata  // Changed to 'reg'
);
    reg [31:0] mem [0:1023]; 
    always @(addr or mem_read) begin
        if (mem_read)
            rdata = mem[addr[31:2]];
        else
            rdata = 32'b0;
    end

    always @(posedge clk) begin
        if (mem_write)
        mem[addr[31:2]] <= wdata;
    end


endmodule

// Branch Compare //

`timescale 1ns/1ps
 module Branch_compare (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [2:0]  funct3,
    output reg    take_branch
);
    wire signed [31:0] s_rs1 = rs1;
    wire signed [31:0] s_rs2 = rs2;

    always @(*) begin
        case (funct3)
            3'b000: take_branch = (rs1 == rs2);          // BEQ
            3'b001: take_branch = (rs1 != rs2);          // BNE
            3'b100: take_branch = (s_rs1 < s_rs2);       // BLT
            3'b101: take_branch = (s_rs1 >= s_rs2);      // BGE
            3'b110: take_branch = (rs1 < rs2);           // BLTU
            3'b111: take_branch = (rs1 >= rs2);          // BGEU
            default: take_branch = 1'b0;
        endcase
    end
endmodule

        

