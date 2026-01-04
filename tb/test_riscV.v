`timescale 1ns/1ps
module test_riscV;

    reg clk;
    reg rst;

    // DUT
    riscv_cpu dut (
        .clk(clk),
        .rst(rst)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end


    initial begin

        $dumpfile("wave.vcd");
        $dumpvars(0,test_riscV);
        rst = 1;

        // -----------------------------
        // LOAD PROGRAM INTO IMEM
        // -----------------------------
        dut.IMEM.mem[0]  = 32'h040300b7;  // lui x1, 0x4030
        dut.IMEM.mem[1]  = 32'h20108093;  // addi x1, x1, 513
        dut.IMEM.mem[2]  = 32'h08070137;  // lui x2, 0x8070
        dut.IMEM.mem[3]  = 32'h60510113;  // addi x2, x2, 1541
        dut.IMEM.mem[4]  = 32'h00400193;  // addi x3, x0, 4 
        dut.IMEM.mem[5]  = 32'h2020850b;  // mac x10, x1, x2      x2 = { 05, 06, 07, 08 } (INT8 packed), x1 = { 01, 02, 03, 04 } (INT8 packed)
        dut.IMEM.mem[6]  = 32'hfff18193;  // addi x3, x3, -1
        dut.IMEM.mem[7]  = 32'hfe019ce3;  // bne x3, x0, -8
        dut.IMEM.mem[8]  = 32'h00a02023;  // sw x10, 0(x0)
        dut.IMEM.mem[9]  = 32'h0180066f;  // jal x12, 24

        // Release reset
        #20 rst = 0;

        // Run simulation
        #700;

        // ----------------------------- 
        // SELF CHECK 
        // -----------------------------

        if (dut.DMEM.mem[0] !== 32'd280) begin 
            $display(" FAIL: MAC RESULT = %0d", dut.DMEM.mem[0]); 
        end else begin 
            $display(" PASS: MAC RESULT = %0d", dut.DMEM.mem[0]); 
        end

        if (dut.RF.regs[3] !== 32'd0) begin
            $display(" TEST FAIL: Loop counter x3 = %0d", dut.RF.regs[3]);
        end else begin
            $display(" TEST PASS: Loop counter x3 = %0d", dut.RF.regs[3]);
        end

        print_regs();
        $finish;
    end

    always @(posedge clk) begin
        if (!rst && dut.RF.regs[0] !== 32'd0) begin
             $error("x0 register modified! ISA violation");
        end
   end

    // -------------------------
    // Display All Regs
    // -------------------------

    task print_regs;
        integer i;
        begin
            $display("============== Register File Dump ==============");
            for (i = 0; i < 32; i = i+1) begin
                $display("x%0d = %h", i, dut.RF.regs[i]);
            end
            $display("================================================");
        end
    endtask

endmodule

