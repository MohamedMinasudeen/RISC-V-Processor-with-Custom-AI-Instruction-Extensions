`timescale 1ns/1ps
module mac_relu_i8_acc (
    input  wire        clk,
    input  wire        rst,
    input  wire        mac_en,     // accumulate enable
    input  wire        clr_acc,    // clear accumulator
    input  wire [31:0] rs1,        // packed INT8s
    input  wire [31:0] rs2,
    output reg  [31:0] acc_out,    // accumulator result (registered)
    output wire [31:0] rd          // ReLU(output) COMBINATIONAL
);

    // 1. Unpack lanes
    wire signed [7:0]  A0, A1, A2, A3;
     assign A0 = rs1[7:0];
     assign A1 = rs1[15:8];
     assign A2 = rs1[23:16];
     assign A3 = rs1[31:24];

    wire signed [7:0]  B0, B1, B2, B3;
     assign B0 = rs2[7:0];
     assign B1 = rs2[15:8];
     assign B2 = rs2[23:16];
     assign B3 = rs2[31:24];

     wire signed [15:0]  P0, P1, P2, P3;
    // 2. Products
    assign P0 = A0 * B0;
    assign P1 = A1 * B1;
    assign P2 = A2 * B2;
    assign P3 = A3 * B3;

    // 3. Dot product
    wire signed [31:0] sum ;
    assign sum = P0 + P1 + P2 + P3;

    // 4. NEXT accumulator value (what we want this cycle)
    wire signed [31:0] acc_next;
    assign acc_next =(clr_acc) ? 32'sd0 : (mac_en)  ? (acc_out + sum) : acc_out;

    // 5. Registered accumulator
    always @(posedge clk or posedge rst) begin
        if (rst)
            acc_out <= 32'd0;
        else 
            acc_out <= acc_next;
    end

    // 6. ReLU on *next* value, COMBINATIONAL
    assign rd = (acc_next < 0) ? 32'd0 : acc_next;

endmodule