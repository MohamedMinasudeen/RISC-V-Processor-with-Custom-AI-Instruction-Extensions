`timescale 1ns/1ps
module riscv_cpu (
    input  wire clk,
    input  wire rst
);

////////////////////////////////////////////////////////////
// IF STAGE
////////////////////////////////////////////////////////////

wire [31:0] pc, pc_next;
wire        pc_write, ifid_write;

wire branch_taken,pc_write_en;
wire [31:0] branch_target,jump_target;

ProgramCounter PC (
    .clk(clk),
    .rst(rst),
    .pc_write(pc_write_en),
    .pc_next(pc_next),
    .pc(pc)
);

assign pc_write_en = pc_write ;
assign pc_next =
    IDEX_jump    ? jump_target   :
    branch_taken ? branch_target :
                   (pc + 32'd4);




wire [31:0] instr;
Instr_mem IMEM (
    .addr(pc[31:2]),
    .instr(instr)
);

wire ctrl_flush = branch_taken || IDEX_jump;

// IF/ID pipeline //

reg [31:0] IFID_instr, IFID_pc;
always @(posedge clk) begin
    if (rst ) begin
        IFID_instr <= 32'b0;
        IFID_pc    <= 32'b0;
    end else if (ctrl_flush) begin
        IFID_instr <= 32'b0;
    end else begin
        IFID_instr <= instr;
        IFID_pc    <= pc;
    end
end


////////////////////////////////////////////////////////////
// ID STAGE
////////////////////////////////////////////////////////////

wire [6:0] opcode = IFID_instr[6:0];
wire [4:0] rs1    = IFID_instr[19:15];
wire [4:0] rs2    = IFID_instr[24:20];
wire [4:0] rd     = IFID_instr[11:7];
wire [2:0] funct3 = IFID_instr[14:12];
wire [6:0] funct7 = IFID_instr[31:25];

// I-type immediate (addi, lw, jalr)
wire [31:0] imm_i = {{20{IFID_instr[31]}}, IFID_instr[31:20]};

// S-type immediate (sw)
wire [31:0] imm_s = {{20{IFID_instr[31]}},
                     IFID_instr[31:25],
                     IFID_instr[11:7]};

// U-type immediate (lui, auipc)
wire [31:0] imm_u = {IFID_instr[31:12], 12'b0};

// B-type immediate 
wire [31:0] imm_b = { {20{IFID_instr[31]}}, // Sign extension
                      IFID_instr[7],         // imm[11]
                      IFID_instr[30:25],     // imm[10:5]
                      IFID_instr[11:8],      // imm[4:1]
                      1'b0                   // Implicit imm[0]
                    };

// J-type immediate 
wire [31:0] imm_j = { {12{IFID_instr[31]}}, // Sign extension (fills bits 31:20)
                      IFID_instr[19:12],     // imm[19:12]
                      IFID_instr[20],        // imm[11]
                      IFID_instr[30:21],     // imm[10:1]
                      1'b0                   // imm[0]
                    };

reg  [31:0] imm_sel;

always @(*) begin
    case (opcode)
        7'b0000011, // LOAD (lw)
        7'b0010011, // OP-IMM (addi)
        7'b1100111: // JALR
            imm_sel = imm_i;

        7'b0100011: // STORE (sw)
            imm_sel = imm_s;

        7'b1100011: // BRANCH (beq, bne)
            imm_sel = imm_b;

        7'b0110111, // LUI
        7'b0010111: // AUIPC
            imm_sel = imm_u;

        7'b1101111: // JAL
            imm_sel = imm_j;

        default:    // R-type or unused
            imm_sel = 32'b0;
    endcase
end


// Control
wire reg_write, alu_src, mem_read, mem_write, mem_to_reg, branch,jump;
wire alu_en,mac_en;
wire [1:0] alu_op;

Control_unit CU (
    .opcode(opcode),
    .funct3(funct3),
    .funct7(funct7),
    .reg_write(reg_write),
    .alu_src(alu_src),
    .mem_read(mem_read),
    .mem_write(mem_write),
    .mem_to_reg(mem_to_reg),
    .branch(branch),
    .jump(jump),
    .mac_en(mac_en),
    .alu_en(alu_en),
    .alu_op(alu_op)
);

// Register file
wire [31:0] rs1_data, rs2_data;
wire [31:0] WB_data;
wire [4:0]  WB_rd;
wire        WB_reg_write;

Regfile RF (
    .clk(clk),
    .rst(rst),
    .we(WB_reg_write),
    .rs1(rs1),
    .rs2(rs2),
    .rd(WB_rd),
    .rd_data(WB_data),
    .rs1_data(rs1_data),
    .rs2_data(rs2_data)
);

////////////////////////////////////////////////////////////
// HAZARD DETECTION (load-use)
////////////////////////////////////////////////////////////
wire stall, idex_flush;
reg  [4:0] IDEX_rd;
reg        IDEX_mem_read;

Hazard_detection_unit HDU (
    .ID_rs1(rs1),
    .ID_rs2(rs2),
    .EX_rd(IDEX_rd),
    .EX_memread(IDEX_mem_read),
    .stall(stall),
    .pc_write(pc_write),
    .ifid_write(ifid_write),
    .idex_flush(idex_flush)
);

////////////////////////////////////////////////////////////
// ID/EX PIPELINE
////////////////////////////////////////////////////////////
reg [31:0] IDEX_rs1, IDEX_rs2, IDEX_pc; 
reg  [31:0]  IDEX_imm;
reg [4:0]  IDEX_rs1_addr, IDEX_rs2_addr;
reg [2:0]  IDEX_funct3;
reg [6:0]  IDEX_funct7;
reg        IDEX_reg_write, IDEX_mem_write, IDEX_mem_to_reg;
reg        IDEX_branch, IDEX_alu_src,IDEX_jump,IDEX_alu_en,IDEX_mac_en;
reg [1:0]  IDEX_alu_op;
reg [6:0] IDEX_opcode;


always @(posedge clk) begin
    if (rst || idex_flush || ctrl_flush   ) begin
        IDEX_rs1 <= 0; IDEX_rs2 <= 0; IDEX_imm <= 0; IDEX_pc <= 0;
        IDEX_rs1_addr <= 0; IDEX_rs2_addr <= 0;
        IDEX_rd <= 0; IDEX_funct3 <= 0; IDEX_funct7 <= 0;
        IDEX_reg_write <= 0; IDEX_mem_read <= 0;
        IDEX_mem_write <= 0; IDEX_mem_to_reg <= 0;
        IDEX_branch <= 0; IDEX_alu_src <= 0; IDEX_alu_op <= 0;
        IDEX_jump<=0;IDEX_opcode <= 0;
        IDEX_mac_en<=0;IDEX_alu_en<=0;
    end else begin
        IDEX_rs1 <= rs1_data;
        IDEX_rs2 <= rs2_data;
        IDEX_rs1_addr <= rs1;
        IDEX_rs2_addr <= rs2;
        IDEX_imm <= imm_sel;
        IDEX_pc  <= IFID_pc;
        IDEX_rd  <= rd;
        IDEX_funct3 <= funct3;
        IDEX_funct7 <= funct7;
        IDEX_reg_write <= reg_write;
        IDEX_mem_read  <= mem_read;
        IDEX_mem_write <= mem_write;
        IDEX_mem_to_reg <= mem_to_reg;
        IDEX_branch <= branch;
        IDEX_alu_src <= alu_src;
        IDEX_alu_op  <= alu_op;
        IDEX_jump   <= jump;
        IDEX_mac_en <=mac_en;
        IDEX_alu_en <=alu_en;
        IDEX_opcode <= opcode;
    end
end

////////////////////////////////////////////////////////////
// EX STAGE + FORWARDING
////////////////////////////////////////////////////////////
wire [1:0] forwardA, forwardB;

assign forwardA =
    (EXMEM_reg_write && EXMEM_rd != 0 && EXMEM_rd == IDEX_rs1_addr) ? 2'b10 :
    (MEMWB_reg_write && MEMWB_rd != 0 && MEMWB_rd == IDEX_rs1_addr) ? 2'b01 : 2'b00;

assign forwardB =
    (EXMEM_reg_write && EXMEM_rd != 0 && EXMEM_rd == IDEX_rs2_addr) ? 2'b10 :
    (MEMWB_reg_write && MEMWB_rd != 0 && MEMWB_rd == IDEX_rs2_addr) ? 2'b01 :
    2'b00;

reg [31:0] alu_in1, alu_in2;
always @(*) begin
    alu_in1 = (forwardA == 2'b10) ? EXMEM_final_res :
              (forwardA == 2'b01) ? WB_data : IDEX_rs1;

    alu_in2 = (forwardB == 2'b10) ? EXMEM_final_res :
              (forwardB == 2'b01) ? WB_data : IDEX_rs2;
end

wire [3:0] alu_ctrl;
Alu_CtrlUnit ALUCTRL (
    .funct3(IDEX_funct3),
    .funct7(IDEX_funct7),
    .alu_op(IDEX_alu_op),
    .alu_ctrl(alu_ctrl)
);

wire [31:0] alu_b = IDEX_alu_src ? IDEX_imm : alu_in2;
wire [31:0] alu_out;
wire zero;

Alu ALU (
    .a(alu_in1),
    .b(alu_b),
    .ctrl(alu_ctrl),
    .acc(alu_out),
    .zero_flag(zero)
);


wire take_branch;
Branch_compare BC (
    .rs1(alu_in1),
    .rs2(alu_in2),
    .funct3(IDEX_funct3),
    .take_branch(take_branch)
);

wire [31:0] mac_out;
reg [31:0] mac_acc_out;
wire clr_acc = rst;
mac_relu_i8_acc MAC(
    .mac_en(IDEX_mac_en),
    .clk(clk),
    .rst(rst),
    .clr_acc(clr_acc),
    .acc_out(mac_acc_out),
    .rs1(alu_in1),
    .rs2(alu_in2),
    .rd(mac_out)
);
 


wire [31:0] final_res;
assign final_res = IDEX_mac_en ? mac_out : IDEX_alu_en ? alu_out : 32'b0 ;

wire [31:0] pc_plus4_ex = IDEX_pc + 32'd4;
assign jump_target =
    (IDEX_jump && IDEX_opcode == 7'b1101111) ?        // JAL
        (IDEX_pc + IDEX_imm) :
    (IDEX_jump && IDEX_opcode == 7'b1100111) ?        // JALR
       ((alu_in1 + IDEX_imm) & ~32'b1) :
        32'b0;

assign branch_taken  = IDEX_branch && take_branch;
assign branch_target = IDEX_pc + $signed(IDEX_imm);




////////////////////////////////////////////////////////////
// EX/MEM PIPELINE
////////////////////////////////////////////////////////////
reg [31:0] EXMEM_final_res, EXMEM_rs2;
reg [4:0]  EXMEM_rd;
reg        EXMEM_reg_write, EXMEM_mem_read, EXMEM_mem_write, EXMEM_mem_to_reg;

reg [31:0] EXMEM_pc_plus4;
reg        EXMEM_jump;

always @(posedge clk) begin
    if (rst || branch_taken ) begin
        EXMEM_final_res <= 0; EXMEM_rs2 <= 0; EXMEM_rd <= 0;
        EXMEM_reg_write <= 0; EXMEM_mem_read <= 0;
        EXMEM_mem_write <= 0; EXMEM_mem_to_reg <= 0;
        EXMEM_pc_plus4 <= 0; EXMEM_jump <= 0;

    end else begin
        EXMEM_final_res <= final_res;
        EXMEM_rs2 <= alu_in2; 
        EXMEM_rd  <= IDEX_rd;
        EXMEM_reg_write <= IDEX_reg_write;
        EXMEM_mem_read  <= IDEX_mem_read;
        EXMEM_mem_write <= IDEX_mem_write;
        EXMEM_mem_to_reg <= IDEX_mem_to_reg;
        EXMEM_pc_plus4 <= pc_plus4_ex;
        EXMEM_jump     <= IDEX_jump;

    end
end

////////////////////////////////////////////////////////////
// MEM STAGE
////////////////////////////////////////////////////////////
wire [31:0] mem_rdata;

Data_mem DMEM (
    .clk(clk),
    .mem_read(EXMEM_mem_read),
    .mem_write(EXMEM_mem_write),
    .addr(EXMEM_final_res),
    .wdata(EXMEM_rs2),
    .rdata(mem_rdata)
);

////////////////////////////////////////////////////////////
// MEM/WB PIPELINE
////////////////////////////////////////////////////////////
reg [31:0] MEMWB_alu_out, MEMWB_mem;
reg [4:0]  MEMWB_rd;
reg        MEMWB_reg_write, MEMWB_mem_to_reg;
reg [31:0] MEMWB_pc_plus4;
reg        MEMWB_jump;


always @(posedge clk) begin
    if (rst) begin
        MEMWB_alu_out <= 0; MEMWB_mem <= 0;
        MEMWB_rd <= 0; MEMWB_reg_write <= 0; MEMWB_mem_to_reg <= 0;
        MEMWB_pc_plus4 <= 0 ; MEMWB_jump  <= 0;

    end else begin
        MEMWB_alu_out <= EXMEM_final_res;
        MEMWB_mem <= mem_rdata;
        MEMWB_rd <= EXMEM_rd;
        MEMWB_reg_write <= EXMEM_reg_write;
        MEMWB_mem_to_reg <= EXMEM_mem_to_reg;
        MEMWB_pc_plus4 <= EXMEM_pc_plus4;
        MEMWB_jump     <= EXMEM_jump;

    end
end

assign WB_data = MEMWB_jump ? MEMWB_pc_plus4 : MEMWB_mem_to_reg ? 
        MEMWB_mem :  MEMWB_alu_out;

assign WB_rd        = MEMWB_rd;
assign WB_reg_write = MEMWB_reg_write;

endmodule
