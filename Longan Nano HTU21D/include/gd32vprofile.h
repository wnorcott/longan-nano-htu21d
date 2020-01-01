/*
 * GD32V profiling library
 * 1/1/2020  W. Norcott created
 */
#ifndef _GD32V_PROFILE_H_
#define _GD32V_PROFILE_H_
#include <stdint.h>
extern uint32_t enable_mcycle_minstret();
// asm volatile("csrr %0, 0xb00 : =r(__tmp));
#define read_csr(csr_reg) ({ \
    unsigned long __tmp; \
    asm volatile ("csrr %0, " #csr_reg : "=r"(__tmp)); \
    __tmp; \
})

/*
    Constant number of machine instructions used by the instructionsExecuted
    function.   This is computed by the function instructionsExecutedOverhead
    and turned into a constant, which is then subtracted from the result
    to zero out the overhead of instructionsExecuted
    */
#define INSTRUCTIONS_EXECUTED_OVERHEAD 0x0a

/* Return number of instructions executed since startup as a 64 bit value
   */
uint64_t instructionsExecuted();
/* Return number of instruction clock cycle ticks since startup as a 64 bit value
 */
uint64_t instructionCycles();
/*  Return instruction overhead of instructionsExecuted function by calling it twice
    in a row and subtracting the two readings to determine cost of instructionsExecuted()
   */
uint64_t instructionsExecutedOverhead();
#endif

