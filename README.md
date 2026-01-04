# RISC-V-Processor-with-Custom-AI-Instruction-Extensions


## Overview
This project implements a **32-bit RISC-V (RV32I) CPU** using **synthesizable Verilog**.
The design includes a **custom packed INT8 Multiply–Accumulate (MAC) instruction**
to demonstrate RTL design, verification, and debugging skills relevant to
**ASIC / VLSI fresher roles**.

The CPU is verified using a **directed, self-checking RTL testbench** with
waveform-based analysis.

---

## Key Features
- RV32I base instruction support
- Modular RTL design
- Custom SIMD-style packed INT8 MAC instruction
- Loop and branch execution
- ISA-compliant register file (`x0` is hardwired to zero)
- Self-checking verification environment

---

## Custom MAC Instruction

The MAC instruction is implemented as a **custom RISC-V instruction** using a
CUSTOM opcode. It follows an **R-type instruction format** and performs a
SIMD-style packed INT8 multiply–accumulate operation.


### Assembly Syntax
MAC rd, rs1, rs2

yaml
Copy code

- `rs1` : Source register containing packed INT8 operands (A)
- `rs2` : Source register containing packed INT8 operands (B)
- `rd`  : Destination register (accumulator)

---

### Instruction Encoding (R-Type Format)

|31:   25| 24 :  20| 19  : 15| 14  : 12 |11  :  7 |6  :  0|
|------------|-------|-------|-------|-------|--------|
|   funct7   |  rs2  |  rs1  | funct3|   rd  | opcode |



---

### Encoding Details

| Field   | Value      | Description |
|--------|------------|-------------|
| opcode | `0001011`  | CUSTOM-0 opcode (RISC-V extension) |
| funct3 | `000`      | MAC operation select |
| funct7 | `0000001`  | MAC function identifier |
| rs1    | Register   | Packed INT8 operands A |
| rs2    | Register   | Packed INT8 operands B |
| rd     | Register   | Accumulator destination |


---

## Verification Strategy
The design is verified using a **directed RTL testbench** with:

- Deterministic clock and reset generation
- Instruction memory initialization in testbench
- Bounded simulation runtime
- Self-checking PASS / FAIL conditions
- Verification of:
  - Data memory result
  - Register file contents
  - Loop termination
  - ISA constraint (x0 register protection)
- Debug support using GTKWave waveforms

---

## Waveform Snapshot
The waveform below shows execution of the custom MAC instruction,
including loop iterations, accumulator update, and final memory store.

![MAC Instruction Waveform](rtl)

---

## Tools Used
- Verilog-2005
- Icarus Verilog
- GTKWave

---

## How to Run Simulation
```bash
iverilog -Wall -g2012 *.v -o sim
vvp sim
gtkwave wave.vcd
