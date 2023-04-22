use tock_registers::{registers::{ReadWrite, WriteOnly}, register_bitfields, register_structs};

// 寄存器结构定义和映射描述
pub const PL061REGS: *mut PL061Regs = (0x0903_0000) as *mut PL061Regs;

// 寄存器位级描述
register_bitfields![
    u32,
    // PrimeCell GPIO interrupt mask
    pub GPIOIE [
        IO3 OFFSET(3) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1
        ]
    ],
];

// 寄存器结构定义和映射描述
register_structs! {
    pub PL061Regs {
        (0x000 => __reserved_0),
        (0x410 => pub ie: ReadWrite<u32, GPIOIE::Register>),
        (0x414 => __reserved_1),
        (0x41c => pub ic: WriteOnly<u32>),
        (0x420 => @END),
    }
}