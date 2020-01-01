
#include "gd32vf103_libopt.h"
#include "gd32vf103.h"
#include "gd32vprofile.h"

// asm volatile("csrr %0, 0xb00 : =r(__tmp));
#define read_csr(csr_reg) ({ \
    unsigned long __tmp; \
    asm volatile ("csrr %0, " #csr_reg : "=r"(__tmp)); \
    __tmp; \
})

extern uint32_t enable_mcycle_minstret();
 
/* Return number of instructions executed since startup as a 64 bit value
   It will be instruction count when the function is called
   */
uint64_t instructionsExecuted()
{
    // read only copies of the number of instructions successfully executed since reset
    // it is a 64 bit quantity that is stored in two 32-bit registers
    // it increments after each instruction sucessfully executes
    uint32_t instret = read_csr(0xC02);
    uint64_t instreth = read_csr(0xC82);
    // pack instruction countback into a 64 bit quantity
    uint64_t instructions = (instreth << 32) | instret;
    return instructions;  
}

/* Return number of instructions clock ticks since startup as a 64 bit value
   It will be instruction count when the function is called
   */
uint64_t instructionCycles()
{

    //read-only copies of the number of instruction clock ticks since reset
    //the cycle register is 64 bits wide and stored in two 32-bit registers
    //it increments on the rising edge of the instruction clock
    unsigned long cycle = read_csr(0xC00);
    unsigned long cycleh = read_csr(0xC80);
    // pack instruction countback into a 64 bit quantity
    uint64_t cycles = cycleh;
    cycles = (cycles << 32) | cycle;
    return cycles;     
}

/* compute the overhead of the  instructionsExecuted function twice in a row
    and subtracting the two values, this is the overhead of the function itself
   */
uint64_t instructionsExecutedOverhead()
{
    /*
     instructionsExecuted returns a bigger number each time called so
    difference is always a positive number */
    uint64_t once = instructionsExecuted();
    return instructionsExecuted() - once;
}

