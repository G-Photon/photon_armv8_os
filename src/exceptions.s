// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Copyright (c) 2018-2021 Andre Richter <andre.o.richter@gmail.com>

.extern el1_sp0_sync
.extern el1_sp0_irq
.extern el1_sp0_fiq
.extern el1_sp0_error
.extern el1_sync
.extern el1_irq
.extern el1_fiq
.extern el1_error
.extern el0_sync
.extern el0_irq
.extern el0_fiq
.extern el0_error
.extern el0_32_sync
.extern el0_32_irq
.extern el0_32_fiq
.extern el0_32_error

//--------------------------------------------------------------------------------------------------
// Definitions
//--------------------------------------------------------------------------------------------------

/// Call the function provided by parameter `\handler` after saving the exception context. Provide
/// the context as the first parameter to '\handler'.
.equ CONTEXT_SIZE, 264

.section .text.exceptions

.macro EXCEPTION_VECTOR handler
    sub sp, sp, #CONTEXT_SIZE

// store general purpose registers
    stp x0, x1, [sp, #16 * 0]
    stp x2, x3, [sp, #16 * 1]
    stp x4, x5, [sp, #16 * 2]
    stp x6, x7, [sp, #16 * 3]
    stp x8, x9, [sp, #16 * 4]
    stp x10, x11, [sp, #16 * 5]
    stp x12, x13, [sp, #16 * 6]
    stp x14, x15, [sp, #16 * 7]
    stp x16, x17, [sp, #16 * 8]
    stp x18, x19, [sp, #16 * 9]
    stp x20, x21, [sp, #16 * 10]
    stp x22, x23, [sp, #16 * 11]
    stp x24, x25, [sp, #16 * 12]
    stp x26, x27, [sp, #16 * 13]
    stp x28, x29, [sp, #16 * 14]

// store exception link register and saved processor state register
    mrs x0, elr_el1
    mrs x1, spsr_el1
    stp x0, x1, [sp, #16 * 15]

// store link register which is x30
    str x30, [sp, #16 * 16]
    mov x0, sp

// call exception handler
    bl \handler

// exit exception
    b .exit_exception
.endm


//--------------------------------------------------------------------------------------------------
// Private Code
//--------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// The exception vector table.
//------------------------------------------------------------------------------
/** When an exception occurs, the processor must execute handler code that corresponds to the exception.
The location in memory where the handler is stored is called the exception vector. In the ARM architecture,
exception vectors are stored in a table, called the exception vector table.

Each Exception level has its own vector table, that is, there is one for each of EL3, EL2, and EL1. The table contains
instructions to be executed, rather than a set of addresses. These would normally be branch instructions that direct the
core to the full exception handler.

The exception vector table for EL1, for example, holds instructions for handling all types of exception that can occur at EL1,
Vectors for individual exceptions are at fixed offsets from the beginning of the table. The virtual address of each table base
is set by the Vector Based Address Registers: VBAR_EL3, VBAR_EL2 and VBAR_EL1.

Each entry in the vector table is 16 instructions long (in ARMv7-A and AArch32, each entry is only 4 bytes). This means that in
AArch64 the top-level handler can be written directly in the vector table.

The base address is given by VBAR_ELn and each entry has a defined offset from this base address. Each table has 16 entries,
with each entry being 128 bytes (32 instructions) in size. The table effectively consists of 4 sets of 4 entries. Which entry
is used depends on several factors:

The type of exception (SError, FIQ, IRQ, or Synchronous)
If the exception is being taken at the same Exception level, the stack pointer to be used (SP0 or SPn)
If the exception is being taken at a lower Exception level, the Execution state of the next lower level (AArch64 or AArch32).
*/



.section .text.exceptions_vector_table
// Export a symbol for the Rust code to use.
.globl exception_vector_table
exception_vector_table:

.org 0x0000
    EXCEPTION_VECTOR el1_sp0_sync

.org 0x0080
    EXCEPTION_VECTOR el1_sp0_irq

.org 0x0100
    EXCEPTION_VECTOR el1_sp0_fiq

.org 0x0180
    EXCEPTION_VECTOR el1_sp0_error

.org 0x0200
    EXCEPTION_VECTOR el1_sync

.org 0x0280
    EXCEPTION_VECTOR el1_irq

.org 0x0300
    EXCEPTION_VECTOR el1_fiq

.org 0x0380
    EXCEPTION_VECTOR el1_error

.org 0x0400
    EXCEPTION_VECTOR el0_sync

.org 0x0480
    EXCEPTION_VECTOR el0_irq

.org 0x0500
    EXCEPTION_VECTOR el0_fiq

.org 0x0580
    EXCEPTION_VECTOR el0_error

.org 0x0600
    EXCEPTION_VECTOR el0_32_sync

.org 0x0680
    EXCEPTION_VECTOR el0_32_irq

.org 0x0700
    EXCEPTION_VECTOR el0_32_fiq

.org 0x0780
    EXCEPTION_VECTOR el0_32_error

.org 0x0800

.exit_exception:
// restore link register
    ldr x30, [sp, #16 * 16]

// restore exception link register and saved processor state register
    ldp x0, x1, [sp, #16 * 15]
    msr elr_el1, x0
    msr spsr_el1, x1

// restore general purpose registers
    ldp x28, x29, [sp, #16 * 14]
    ldp x26, x27, [sp, #16 * 13]
    ldp x24, x25, [sp, #16 * 12]
    ldp x22, x23, [sp, #16 * 11]
    ldp x20, x21, [sp, #16 * 10]
    ldp x18, x19, [sp, #16 * 9]
    ldp x16, x17, [sp, #16 * 8]
    ldp x14, x15, [sp, #16 * 7]
    ldp x12, x13, [sp, #16 * 6]
    ldp x10, x11, [sp, #16 * 5]
    ldp x8, x9, [sp, #16 * 4]
    ldp x6, x7, [sp, #16 * 3]
    ldp x4, x5, [sp, #16 * 2]
    ldp x2, x3, [sp, #16 * 1]
    ldp x0, x1, [sp, #16 * 0]

// restore stack pointer
    add sp, sp, #CONTEXT_SIZE
    eret