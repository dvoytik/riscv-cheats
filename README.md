# RISC-V Instruction Set Cheat Sheet

**WARNING**: expect mistakes

## References :
https://riscv.org/technical/specifications/  
https://github.com/riscv-non-isa/riscv-asm-manual/blob/master/riscv-asm.md

## Definitions, Abbreviations

`EEI` - Execution Environment Interface  
`RVWMO` - RISC-V Weak Memory Ordering  
`imm<N>`, `off<N>` - signed immediate (or offset), `<N>` bit constant (imm5, imm12, imm20, imm32, off12, off20)  
`uimm<N>` - unsigned immediate, `<N>` bit constant (uimm5, uimm20)  
`rd` - Register, Destination  
`rs` - Register, Source (`rs1`, `rs2`)  
`XLEN` - 32 bit for RV32I, 64 bit for RV64I and 128 bit for RV128I.  

## ISAs
Four standard base ISAs:
 * RV32I - 32 bit address space
 * RV64I
 * RV128I
 * RV32E - reduced RV32I for embedded uCs. 16 integer registers instead of 32.

|   | Extension description                                         
| - | ---------------------------------------------------------------
| I | Base integer (addition, subtraction, load, store, control flow)
| M | Multiplication, division
| A | Atomic instruction extension (read, modify, write)
| F | Single precision floating point
| D | Double precision floating point
| G | IMAFD
| C | Compressed extension. 16 bit instructions. For embedded CPUs.
| H | Hypervisor
| Q | Quad-precision floating point extension

| Word size   | bits     |
| ----------- | -------- |
| halfword    | 16 bits  |
| word        | 32 bits  |
| doubleword  | 64 bits  |
| quadword    | 128 bits |


Instructions 32 bit aligned on 32 bit boundaries.  
IALIGN = 32 or 16 - instruction-address alignment constraint.  

## Instruction format

Parcel - 16 bit in little-endian byte order.
In memory - little endian byte order:
```
Byte address |    00    ||    01    ||   02   ||   03   |
Parcels      |    low bits parcel   || high bits parcel |
             | xxxbbb11 || xxxxxxxx || high bits parcel |
```

## RV32I Base Integer Instruction Set, Version 2.1
40 unique instructions

## RV32I Unprivileged registers
ISA doesn't enforce link and stack pointer register.  
| Reg | ABI | Preserverd? | Description         
| --- | --- | - | ------------------------------
| x0  | zero| - | read as zero, write is ignored
| x1  | ra  | n | ret addr - used to hold subroutine return address (link register) in standard SW convention
| x2  | sp  | y | used to hold stack pointer in standard SW convention
| x3  | gp  | - | global pointer
| x4  | tp  | - | thread pointer
| x5  | t0  | n | temp reg 0, alternate link register
| x6  | t1  | n | temp reg 1
| x7  | t2  | n | temp reg 2
| x8  | s0  | y | saved register 0 or frame pointer
| x9  | s1  | y | saved register 1
| x10 | a0  | n | return value or function argument 0
| x11 | a1  | n | return value or function argument 1
| x12 | a2  | n | function argument 2
| x13 | a3  | n | function argument 3
| x14 | a4  | n | function argument 4
| x15 | a5  | n | function argument 5
| x16 | a6  | n | function argument 6
| x17 | a7  | n | function argument 7
| x18 | s2  | y | saved register 2
| x19 | s3  | y | saved register 3
| x20 | s4  | y | saved register 4
| x21 | s5  | y | saved register 5
| x22 | s6  | y | saved register 6
| x23 | s7  | y | saved register 7
| x24 | s8  | y | saved register 8
| x25 | s9  | y | saved register 9
| x26 | s10 | y | saved register 10
| x27 | s11 | y | saved register 11
| x28 | t3  | n | temp reg 2
| x29 | t4  | n | temp reg 2
| x30 | t5  | n | temp reg 2
| x31 | t6  | n | temp reg 6
| pc  | --- | - | Program counter 

### RISC-V base instructions format
```
R-type: opcode[0:6] rd [7:11]  funct3[12:14] rs1[15:19] rs2[20:24] funct7[25:31]
I-type: opcode[0:6] rd [7:11]  funct3[12:14] rs1[15:19] imm[20:31]
S-type: opcode[0:6] imm[7:11]  funct3[12:14] rs1[15:19] rs2[20:24] imm   [25:31]
U-type: opcode[0:6] rd [11:7]  imm   [12:31]

rs - source
rd - destination
CSR - control and status registers
```

## Integer Register-Immediate instructions

| Instr | Parameters     | Pseudo-code                            | Description          
| ----  | -------------- | -------------------------------------- | -----------------------------------------------
| addi  | rd, rs1, imm12 | rd = rs1 + imm12                       | add imm12. Arithmetic overflow is ignored
| slti  | rd, rs1, imm12 | if rs1 < imm12 then rd = 1 else rd = 0 | set less than imm12; rs is signed
| sltiu | rd, rs1, imm12 | if rs1 < imm12 then rd = 1 else rd = 0 | set less than imm12 ; rs1 is unsigned
| andi  | rd, rs1, imm12 | rd = rs1 & sign_extend(imm12)          | bitwise AND of rs1 with imm12
| ori   | rd, rs1, imm12 | rd = rs1 | sign_extend(imm12)          | bitwise OR of rs1 with imm12
| xori  | rd, rs1, imm12 | rd = rs1 ^ sign_extend(imm12)          | bitwise XOR of  rs1 with imm12
| srli  | rd, rs1, imm5  | rd = rs1 >> imm5                       | shift right logical imm5 (zeros shift in upper bits)
| slli  | rd, rs1, imm5  | rd = rs1 << imm5                       | shift left logical imm5 (zeros shift in lower bits)
| srai  | rd, rs1, imm5  | rd = rs1 >> imm5 ; rd[31] = rs1[31]    | shift right arithmetic imm5 
|       |                |                                        | (sign bit copied in vacated upper bits)
| lui   | rd, uimm20     | rd = uimm20 << 12                      | Load Upper Immediate
| auipc | rd, uimm20     | rd = (uimm20 << 12) + pc               | Add upper immediate to PC


## Integer pseudo instructions

| Pseudo instr | Real instruction | Pseudo-code                         | Description
| ------------ | ---------------  | ----------------------------------- | ------------------------------ |
| nop          | addi x0, x0, 0   |                                     | no operation
| mv rd, rs1   | addi rd, rs1, 0  | rd = rs1                            | move (copy) register rs1 to rd
| seqz d, rs1  | sltiu rd, rs1, 1 | if rs1 == 0 then rd = 1 else rd = 0 | set equal zero
| not rd, rs1  | xori rd, rs1, -1 | rd = ~rs1                           | bitwise NOT

```
li   - load imm32
la rd, symbol (non-PIC) - load absolute address, where delta = symbol - pc
la rd, symbol (PIC)     - load absolute address, where delta = GOT[symbol] - pc
la rd, symbol           - Loadl local address, where delta = symbol - pc

```

```
li rd, imm32
# real two instructions
lui  rd, imm32[31:12]
addi rd, rd, imm32[11:0]

li a0, 0xdeadbeef
# real instructions:
lui a0, 0xdeadb
addi a0, a0, 0xeef

# - load absolute address, where delta = symbol - pc
la rd, symbol (non-PIC) 
# real two instructions
auipc rd, delta[31:12] + delta[11]
addi  rd, rd, delta[11:0]
# rd = pc + (delta[31:12] + delta[11] << 12)
# rd = rd + delta[11:0]

# Example
# stack_top = 8000804c
#        pc = 80000008
la    sp, stack_top 
# delta = 0x8000804c - 0x80000008 = 0x8044
auipc   sp,0x8      # sp = 80000008 + 0x8 << 12 = 0x80008008
addi    sp,sp,0x44  # sp = 0x80008008 + 0x44 = 0x8000804c

# - load absolute address, where delta = GOT[symbol] - pc
la rd, symbol (PIC)     

# - Loadl local address, where delta = symbol - pc
la rd, symbol           

```

## Integer Register-Register Operations

| Instr | Parameters     | Pseudo-code                            | Description          
| ----  | -------------- | -------------------------------------- | -----------------------------------------------
| add   | rd, rs1, rs2   | rd = rs1 + rs2                         | Addition of rs1 and rs2. Overflow is ignored.
|  |  |  | 
|  |  |  | 
|  |  |  | 
|  |  |  | 


```
sub  - substruction of rs2 from rs1. Overflow is ignored.
slt  - set less than
sltu - set less than unsigned
and 
or 
xor 
sll - shift left logical
srl - shift right logical
sra - shift right arithmetic
```

```
# substruction of rs2 from rs1. Overflow is ignored.
sub rd, rs1, rs2
 # rd = rs1 - rs2

# set less than
slt rd, rs1, rs2
 # if rs1 < rs2 then rd = 1 else rd = 0

# set less than unsigned
sltu rd, rs1, rs2
 # if rs1 < rs2 then rd = 1 else rd = 0

and rd, rs1, rs2
 # rd = rs1 & rs2

or rd, rs1, rs2
 # rd = rs1 | rs2

xor rd, rs1, rs2
 # rd = rs1 ^ rs2


sll rd, rs1, rs2
 # rd = rs1 << rs2[4:0]

```

## Integer Register-Register pseudoinstruction

snez - set not qual zero pseudoinstruction

snez rd, rs2 == sltu rd, x0, rs2

```
# set not qual zero pseudoinstruction
snez rd, rs2 # == sltu rd, x0, rs2
# if rs2 > 0 then rd = 1 else rd = 0

```

## Unconditional Jumps

| Instr | Parameters     | Pseudo-code                              | Description          
| ----  | -------------- | ---------------------------------------- | ----------------------------------
| jal   | rd, off20      | rd = pc + 4 ; pc = pc + off20 << 1       | Jump And Link. Addresses +/- 1 MiB.
| jalr  | rd, off12(rs1) | rd = pc + 4 ; pc = rs1 + off12; pc[0] = 0| Jump And Link Regitster
|       |                |                                          | 
|  |  |  | 
|  |  |  | 

Standard calling convention to use link register rd as x1 or x5.  

```
# Jump 32-bit relative to PC
auipc t0, address[31:12]    # load 20-bit higher bits of address + pc
jalr  x1, t0, address[11:0] # jump to t0 + 12-bit low address
 # t0 = address[31:12] + pc
 # x1 = pc + 4
 # pc = t0 + address[11:0]

# Jump any 32 bit address
lui  t0, address[31:12]     # load 20-bit higher bits of address
jalr x1, t0, address[11:0]  # jump to pc + address[31:0]
 # t0 = address[31:12]
 # x1 = pc + 4
 # pc = t0 + address[11:0]

```

## Unconditional Jumps Pseudo Instructions

| Pseudo instr    | Real instruction    | Pseudo-code                       | Description
| --------------- | ------------------- | --------------------------------- | -----------------------
| ret             | jalr x0, 0(x1)      | pc = x1                           | Return from subroutine


```
j - plain unconditional jump pseudoinstruction
jal - also a pseudoverison of jal instruction

j   off20 == jal x0, off20
jal off20 == jal x1, off20

# plain unconditional jump pseudoinstruction
j off20
 # pc = pc + sing_extend(off20)

jal off20 - pseudo instruction
 # x1 = pc + 4         # x1 == ra
 # pc = pc + off20 << 1

```

## Conditional Branches

All branch instruction encode 12-bit sign extended off12 of 2 bytes.
The Conditional branch is +/- 4 KiB.

| Instr | Parameters      | Pseudo-code                             | Description          
| ----  | --------------- | --------------------------------------- | ---------------------
| beq   | rs1, rs2, off12 | if rs1 == rs2 then pc = pc + off12 << 1 | Branch Equal
| bne   | rs1, rs2, off12 | if rs1 != rs2 then pc = pc + off12 << 1 | Branch Not Equal
|  |  |  | 


bne  - branch not equal  
blt  - branch less than  
bltu - Branch Less Than Unsigned  
bge  - Branch Greater Equal  
bgeu - Branch Greater Equal Unsigned  

```

# branch less than
blt rs1, rs2, off12
 # if rs1 < rs2 then pc = pc + off12 << 1

# Branch Less Than Unsigned
bltu rs1, rs2, off12
 # if rs1 < rs2 then pc = pc + off12 << 1

# Branch Greater Equal
bge rs1, rs2, off12
 # if rs1 >= rs2 then pc = pc + off12 << 1

# Branch Greater Equal Unsigned
bgeu rs1, rs2, off12
 # if rs1 >= rs2 then pc = pc + off12 << 1

```

## Conditional Pseudo Instructions

| Pseudo instr    | Real instruction    | Pseudo-code                           | Description
| --------------- | ------------------- | ------------------------------------- | --------------------- 
| bnez rs1, off12 | bne rs1, x0, off12  | if rs1 != 0 then pc = pc + off12 << 1 | Branch Not Equal Zero

```
beqz rs, off12  - branch if == 0
blez rs, off12  - branch if <= 0
bqez rs, off12  - branch if >= 0
bltz rs, off12  - branch if < 0
bgtz rs, off12  - branch if > 0
```

```
beqz rs, off12 == beq rs, x0, off12

blez rs, off12  ==
bqez rs, off12  ==
bltz rs, off12  ==
bgtz rs, off12  ==

```


## Load and Store Instructions

| Instr | Parameters     | Pseudo-code                                 | Description          
| ----  | -------------- | ------------------------------------------- | ---------------------------
| lbu   | rd, off12(rs1) | rd[7:0] = memory[rs1 + sign_extend(off12)]  | Load Byte Unsigned
|       |                | rd[XLEN:8] = 0                              | zero extends rd
| lb    | rd, off12(rs1) | rd[7:0] = memory[rs1 + sign_extend(off12)]  | Load Word, sign extends
|       |                | rd[XLEN:8] = sign                           | sign extends rd
| lw    | rd, off12(rs1) | rd[31:0] = memory[rs1 + sign_extend(off12)] | Load Word, sign extends
|       |                | rd[XLEN:32] = sign                          | sign extends rd
|  |  |  | 

```
lw  - Load Word
lh  - Load Halfword
lhu - Load Word Unsigned
lb  - Load Byte
sw  - Store Word
sh  - Store Halfword
sb  - Store Byte
```

```
# Load Word
lw    rd, off12(rs1)
 # rd = memory[rs1 + sign_extend(off12)]

lh    rd, off12(rs1)
 # rd = sign_extend(memory[rs1 + sign_extend(off12)])

lhu   rd, off12(rs1)
 # rd = zero_extend(memory[rs1 + sign_extend(off12)])

# Store Word
sw    rs2, off12(rs1)
 # memory[rs1 + sign_extend(off12)] = rs2
 # rs1[off12] = rs2

```

## Memory Ordering Instructions

fence 

## Environment Call and Breakpoints

ecall
ebreak

## RV64I Integer Computational Instructions

32-bit word is always sign extended to 64-bit word

```
addiw - add imm12 word
slliw
srliw
sraiw

addw  - add word
subw  - Subtract word 

sllw 
srlw 
sraw 
```

## RV64I Load and Store 

```
ld - Load Doubleword 
sd - Store Doubleword
```

## Multiplication Operations

"M" extension

```
mul     - multiplication, low XLEN bits
mulh    - multiplication, high XLEN bits, signed x signed
mulhu   - multiplication, high XLEN bits, unsigned x unsigned
mulhsu  - multiplication, high XLEN bits, signed rs1, unsigned rs2
mulw    - RV64 instruction
```

```
# full restult - two instructions
mulh rdh, rs1, rs2
mul  rdl, rs1, rs2
 # result is in rdh - [63:32] and rdl - [31:0]
```

```
div
divu 
rem 
remu 
divw
divuw
remw
remuw
```


```
div t0, rs1, rs2
 # t0 = round_to_zero(rs1 / rs2)

rem t1, rs1, rs2
```

## Load-Reserved/Store-Conditional Instructions

"A" Standard Extension for Atomic Instructions, Version 2.1

```
lr.w  - load reserve, word
sc.w  - store conditional, word
lr.d  - loard reserve, doubleword
sc.d  - store conditional, doubleword
```


```
# store conditional, word
sc.w t0, a2, (a0)
 # if success then mem[a0] = a2, t0 = 0; else t0 = not(0x0)
```

Compare and Swap (CAS) example:

```
# a0 holds address of memory location
# a1 holds expected value
# a2 holds desired value
# a0 holds return value, 0 if successful, !0 otherwise
cas:
    lr.w t0, (a0)       # Load original value.
    bne t0, a1, fail    # Doesn’t match, so fail.
    sc.w t0, a2, (a0)   # Try to update.
    bnez t0, cas        # Retry if store-conditional failed.
    li a0, 0            # Set return to success.
    jr ra               # Return.
fail:
    li a0, 1            # Set return to failure.
    jr ra               # Return.
```

## Atomic Memory Operations

```
amoswap.w  - Atomic Memory Operation, Swap, Word
amoswap.d
amoadd.w   - Atomic Memory Operation, Add, Word
amoadd.d   - Atomic Memory Operation, Add, Doubleword 
amoor.w
amoor.d
amoxor.w 
amomax.w
amomaxu.w
amomax.d
amomaxu.d
amomin.w
amomin.d
amominu.w
amominu.d

xx.y.aq - then no later memory operations in this RISC-V harts 
          can be observed to take place before the AMO
xx.y.rl - then other RISC-V harts will not observe the AMO before 
          memory accesses preceding the AMO in this RISC-V hart.
```

```
# a0 contains the address of the lock
    li t0, 1         # Initialize swap value.
again:
    lw t1, (a0)      # Check if lock is held.
  bnez
  amoswap.w.aq t1, t0, (a0) # Attempt to acquire lock.
  bnez         t1, again    # Retry if held.
  # ...
  # Critical section.
  # ...
  amoswap.w.rl x0, x0, (a0) # Release lock by storing 0.
```

## CSR Instructions

`CSR` - Control Status Register. All CSR instructions atomically read-modify-write a single CSR.  
All reads from CSRs are zero extended to XLEN bits before writing to rd.  
If rd == x0 then CSR is not read, i.e. no side effect of reading the CSR.

| Instr  | Parameters     | Pseudo-code                   | Description
| ------ | -------------- | ----------------------------- | ------------------------------
| csrrw  | rd, csr, rs1   | rd = csr, csr = rs1           | CSR Read, Write.
| csrrs  | rd, csr, rs1   | rd = csr, csr = csr \| rs1    | CSR Read, Set bit mask.
| csrrc  | rd, csr, rs1   | rd = csr, csr = csr & ~rs1    | CSR Read, Clear bit mask.
| csrrwi | rd, csr, uimm5 | rd = csr, csr = 0..0 \| uimm5 | CSR Read, Write uimm5
| csrrsi | rd, csr, uimm5 | rd = csr, csr = csr \| uimm5  | CSR Read, Set by 5 bit mask.
| csrrci | rd, csr, uimm5 | rd = csr, csr = csr & ~uimm5  | CSR Read, Clear by 5 bit mask.

csrrsi, csrrci always cause CSR read side effect.

`uimm5` - unsigned 5 bit immediate constant. It's zero extended before writing to a CSR.

## CSR Pseudo instructions:

| Pseudo instr | Real instruction   | Pseudo-code                         | Description
| ------------ | ------------------ | ----------------------------------- | ------------------------------ |
| csrr rd, csr | csrrs rd, csr, x0  | rd = csr                            | CSR, Read
| csrw csr, rs | csrrw x0, csr, rs1 | csr = rs                            | CSR, Write

```
csrwi   writ imm5 to CSR
csrs    set bit in CSR
csrc    clear bit in CSR
csrsi   set #imm5 bit mask in CSR
csrci   clear #imm5 bit mask in CSR

csrwi csr, uimm5 == csrrwi x0, csr, uimm5
csrs  csr, rs1   == csrrs x0, csr, rs1
csrc  csr, rs1   == csrrc x0, csr, rs1
csrsi csr, uimm5 == csrrsi x0, csr, uimm5

```

