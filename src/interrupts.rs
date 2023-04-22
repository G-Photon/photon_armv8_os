use core::ptr;

// GICD和GICC寄存器内存映射后的起始地址
const GICD_BASE: u64 = 0x08000000;
const GICC_BASE: u64 = 0x08010000;

// Distributor
const GICD_CTLR: *mut u32 = (GICD_BASE + 0x0) as *mut u32;//控制Distributor是否向CPU Interface转发中断。
const GICD_ISENABLER: *mut u32 = (GICD_BASE + 0x0100) as *mut u32;//控制对应中断的转发使能行为。
const GICD_ICENABLER: *mut u32 = (GICD_BASE + 0x0180) as *mut u32;//控制对应中断的转发失能行为。
const GICD_ISPENDR: *mut u32 = (GICD_BASE + 0x0200) as *mut u32;//设置一个或一组中断为pending状态。
const GICD_ICPENDR: *mut u32 = (GICD_BASE + 0x0280) as *mut u32;//清除一个或一组中断为pending状态。
const GICD_IPRIORITYR: *mut u32 = (GICD_BASE + 0x0400) as *mut u32;//用于设置中断的优先级。
const GICD_ICFGR: *mut u32 = (GICD_BASE + 0x0c00) as *mut u32;//用于设置中断的触发方式。

const GICD_CTLR_ENABLE: u32 = 1;  /* Enable GICD */
const GICD_CTLR_DISABLE: u32 = 0;     /* Disable GICD */
const GICD_ISENABLER_SIZE: u32 = 32;
const GICD_ICENABLER_SIZE: u32 = 32;
const GICD_ISPENDR_SIZE: u32 = 32;
const GICD_ICPENDR_SIZE: u32 = 32;
const GICD_IPRIORITY_SIZE: u32 = 4;
const GICD_IPRIORITY_BITS: u32 = 8;
const GICD_ICFGR_SIZE: u32 = 16;
const GICD_ICFGR_BITS: u32 = 2;


// CPU Interface
const GICC_CTLR: *mut u32 = (GICC_BASE + 0x0) as *mut u32;//控制CPU interface传给CPU的中断信号。
const GICC_PMR: *mut u32 = (GICC_BASE + 0x0004) as *mut u32;//提供优先级过滤功能
const GICC_BPR: *mut u32 = (GICC_BASE + 0x0008) as *mut u32;//此寄存器用来把8位的优先级字段拆分为组优先级和子优先级，组优先级用来决定中断抢占。
const GICC_IAR: *mut u32 = (GICC_BASE + 0x0c) as *mut u32;//CPU读此寄存器，获得当前中断的interrtup ID。
const GICC_EOIR: *mut u32 = (GICC_BASE + 0x10) as *mut u32;//写此寄存器，表示某中断已经处理完毕。GICC_IAR的值表示当前在处理的中断，把GICC_IAR的值写入GICC_EOIR就表示中断处理完了。

const GICC_CTLR_ENABLE: u32 = 1;
const GICC_CTLR_DISABLE: u32 = 0;
const GICC_PMR_PRIO_LOW: u32 = 0xff;
// Priority Mask Register. interrupt priority filter, Higher priority corresponds to a lower Priority field value.
// 优先级掩码寄存器，中断优先级过滤器，较高优先级对应较低优先级字段值。
const GICC_BPR_NO_GROUP: u32 = 0x00;
// The register defines the point at which the priority value fields split into two parts,
// the group priority field and the subpriority field. The group priority field is used to
// determine interrupt preemption. NO GROUP.
// 优先级分组是将GICC_BPR（Binary PointRegister）分为两个域，组优先级（group priority）和组内优先级（subpriority）。
// 当决定抢占（Preemption）的时候，组优先级相同的中断被视为一样的，不考虑组内优先级。那就意味着在每个优先级组内只能有一个中断被激活。
// 组优先级又被称为抢占级别（preemption level）。这里令其无组优先级。

const TIMER_IRQ: u32 = 30;
const UART0_IRQ: u32 = 33;
const GPIO_IRQ: u32 = 39; // virt.dts interrupts = <0x00 0x07 0x04>; 32 + 0x07 = 39

use core::arch::asm;
pub fn init_gicv2() {
    // 初始化Gicv2的distributor和cpu interface
    // 禁用distributor和cpu interface后进行相应配置
    unsafe {
        ptr::write_volatile(GICD_CTLR, GICD_CTLR_DISABLE);
        ptr::write_volatile(GICC_CTLR, GICC_CTLR_DISABLE);
        ptr::write_volatile(GICC_PMR, GICC_PMR_PRIO_LOW);
        ptr::write_volatile(GICC_BPR, GICC_BPR_NO_GROUP);
    }

    // 启用distributor和cpu interface
    unsafe {
        ptr::write_volatile(GICD_CTLR, GICD_CTLR_ENABLE);
        ptr::write_volatile(GICC_CTLR, GICC_CTLR_ENABLE);
    }    
    // 电平触发
    const ICFGR_LEVEL: u32 = 0;
    // 初始化UART0 中断
    // interrupts = <0x00 0x01 0x04>; SPI, 0x01, level
    set_config(UART0_IRQ, ICFGR_LEVEL); //电平触发
    set_priority(UART0_IRQ, 0); //优先级设定
    // set_core(TIMER_IRQ, 0x1); // 单核实现无需设置中断目标核
    clear(UART0_IRQ); //清除中断请求
    enable(UART0_IRQ); //使能中断


    set_config(TIMER_IRQ, ICFGR_LEVEL); //电平触发
    set_priority(TIMER_IRQ, 0); //优先级设定
    clear(TIMER_IRQ); //清除中断请求
    enable(TIMER_IRQ); //使能中断
    
    // 初始化GPIO中断
    set_config(GPIO_IRQ, ICFGR_LEVEL); //电平触发
    set_priority(GPIO_IRQ, 0); //优先级设定
    clear(GPIO_IRQ); //清除中断请求
    enable(GPIO_IRQ); //使能中断

    // 使能GPIO的poweroff key中断
    use crate::pl061::*;
    unsafe{
        let pl061r: &PL061Regs = &*PL061REGS;

        // 启用pl061 gpio中的3号线中断
        pl061r.ie.write(GPIOIE::IO3::Enabled);
    }

    //配置timer
    unsafe {
        asm!("mrs x1, CNTFRQ_EL0"); //读取系统频率
        asm!("msr CNTP_TVAL_EL0, x1");  //设置定时寄存器
        asm!("mov x0, 1");
        asm!("msr CNTP_CTL_EL0, x0"); //enable=1, imask=0, istatus= 0,
        asm!("msr daifclr, #2");//开启IRQ
    }
    // let mut sec=0;
    // loop {
    //     unsafe {
    //         sec+=1;
    //         asm!("wfi"); // Wait for Interrupt 等待中断，下一次中断发生前都停止在此处
            
    //     }
    //     println!("{} seconds have passed since the program started",sec);
    // }
}

// 使能中断号为interrupt的中断
pub fn enable(interrupt: u32) {
    unsafe {
        ptr::write_volatile(
            GICD_ISENABLER.add((interrupt / GICD_ISENABLER_SIZE) as usize),
            1 << (interrupt % GICD_ISENABLER_SIZE)
        );
    }
}

// 禁用中断号为interrupt的中断
pub fn disable(interrupt: u32) {
    unsafe {
        ptr::write_volatile(
            GICD_ICENABLER.add((interrupt / GICD_ICENABLER_SIZE) as usize),
            1 << (interrupt % GICD_ICENABLER_SIZE)
        );
    }
}

// 清除中断号为interrupt的中断
pub fn clear(interrupt: u32) {
    unsafe {
        ptr::write_volatile(
            GICD_ICPENDR.add((interrupt / GICD_ICPENDR_SIZE) as usize),
            1 << (interrupt % GICD_ICPENDR_SIZE)
        );
    }
}

// 设置中断号为interrupt的中断的优先级为priority
pub fn set_priority(interrupt: u32, priority: u32) {
    let shift = (interrupt % GICD_IPRIORITY_SIZE) * GICD_IPRIORITY_BITS;
    unsafe {
        let addr: *mut u32 = GICD_IPRIORITYR.add((interrupt / GICD_IPRIORITY_SIZE) as usize);
        let mut value: u32 = ptr::read_volatile(addr);
        value &= !(0xff << shift);
        value |= priority << shift;
        ptr::write_volatile(addr, value);
    }
}

// 设置中断号为interrupt的中断的属性为config
pub fn set_config(interrupt: u32, config: u32) {
    let shift = (interrupt % GICD_ICFGR_SIZE) * GICD_ICFGR_BITS;
    unsafe {
        let addr: *mut u32 = GICD_ICFGR.add((interrupt / GICD_ICFGR_SIZE) as usize);
        let mut value: u32 = ptr::read_volatile(addr);
        value &= !(0x03 << shift);
        value |= config << shift;
        ptr::write_volatile(addr, value);
    }
}

use core::arch::global_asm;
global_asm!(include_str!("exceptions.s"));

#[repr(C)]
pub struct ExceptionCtx {
    regs: [u64; 30],
    elr_el1: u64,
    spsr_el1: u64,
    lr: u64,
}

const EL1_SP0_SYNC: &'static str = "EL1_SP0_SYNC";
const EL1_SP0_IRQ: &'static str = "EL1_SP0_IRQ";
const EL1_SP0_FIQ: &'static str = "EL1_SP0_FIQ";
const EL1_SP0_ERROR: &'static str = "EL1_SP0_ERROR";
const EL1_SYNC: &'static str = "EL1_SYNC";
const EL1_IRQ: &'static str = "EL1_IRQ";
const EL1_FIQ: &'static str = "EL1_FIQ";
const EL1_ERROR: &'static str = "EL1_ERROR";
const EL0_SYNC: &'static str = "EL0_SYNC";
const EL0_IRQ: &'static str = "EL0_IRQ";
const EL0_FIQ: &'static str = "EL0_FIQ";
const EL0_ERROR: &'static str = "EL0_ERROR";
const EL0_32_SYNC: &'static str = "EL0_32_SYNC";
const EL0_32_IRQ: &'static str = "EL0_32_IRQ";
const EL0_32_FIQ: &'static str = "EL0_32_FIQ";
const EL0_32_ERROR: &'static str = "EL0_32_ERROR";

// 调用我们的print!宏打印异常信息，你也可以选择打印异常发生时所有寄存器的信息
fn catch(ctx: &mut ExceptionCtx, name: &str) {
    crate::print!(
        "\n  \
        {} @ 0x{:016x}\n\n ",
        name,
        ctx.elr_el1,
    );
}

#[no_mangle]
unsafe extern "C" fn el1_sp0_sync(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_SP0_SYNC);
}

#[no_mangle]
unsafe extern "C" fn el1_sp0_irq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_SP0_IRQ);
}

#[no_mangle]
unsafe extern "C" fn el1_sp0_fiq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_SP0_FIQ);
}

#[no_mangle]
unsafe extern "C" fn el1_sp0_error(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_SP0_ERROR);
}

#[no_mangle]
unsafe extern "C" fn el1_sync(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_SYNC);
}

fn handle_irq_lines(ctx: &mut ExceptionCtx, _core_num: u32, irq_num: u32) {
    if irq_num == TIMER_IRQ {
        handle_timer_irq(ctx);
    }else if irq_num == UART0_IRQ {
        handle_uart0_rx_irq(ctx);
    }else if irq_num == GPIO_IRQ {
        handle_gpio_irq(ctx);
    }
    else{
        catch(ctx, EL1_IRQ);
    }
}

fn handle_timer_irq(_ctx: &mut ExceptionCtx){

    //crate::print!(".");
    // 每2秒产生一次中断
    unsafe {
        asm!("mrs x1, CNTFRQ_EL0");
        //asm!("add x1, x1, x1");
        asm!("msr CNTP_TVAL_EL0, x1");
    }
//    println!("Time interrupts!");

}

use tock_registers::interfaces::{Readable,Writeable};

fn handle_uart0_rx_irq(_ctx: &mut ExceptionCtx){
    use crate::uart_console::pl011::*;

    // crate::print!("R");
    unsafe{
        // pl011 device registers
        let pl011r: &PL011Regs = &*PL011REGS;

        let mut flag = pl011r.fr.read(UARTFR::RXFE);
        while flag != 1 {
            
            let value = pl011r.dr.read(UARTDR::DATA);
            //while value != 13 {
                //value =pl011r.dr.read(UARTDR::DATA);
                //crate::print!("{}", value as u8 as char);
                crate::print!("{}",value as u8 as char);
            //}
            flag = pl011r.fr.read(UARTFR::RXFE);
        }
    }
}

fn handle_gpio_irq(_ctx: &mut ExceptionCtx){
    use crate::pl061::*;
    crate::println!("power off!\n");
    unsafe {
        let pl061r: &PL061Regs = &*PL061REGS;

        // 清除中断信号 此时get到的应该是0x8
        pl061r.ic.set(pl061r.ie.get());
        // 关机
        asm!("mov w0, #0x18");
        asm!("hlt #0xF000");
    }
}

#[no_mangle]
unsafe extern "C" fn el1_irq(ctx: &mut ExceptionCtx) {
    // reads this register to obtain the interrupt ID of the signaled interrupt.
    // This read acts as an acknowledge for the interrupt.
    // 中断确认
    let value: u32 = ptr::read_volatile(GICC_IAR);
    let irq_num: u32 = value & 0x1ff;
    let core_num: u32 = value & 0xe00;

    // 实际处理中断
    handle_irq_lines(ctx, core_num, irq_num);
    // catch(ctx, EL1_IRQ);

    // A processor writes to this register to inform the CPU interface either:
    // • that it has completed the processing of the specified interrupt
    // • in a GICv2 implementation, when the appropriate GICC_CTLR.EOImode bit is set to 1, to indicate that the interface should perform priority drop for the specified interrupt.
    // 标记中断完成，清除相应中断位
    ptr::write_volatile(GICC_EOIR, core_num | irq_num);
    clear(irq_num);
}

#[no_mangle]
unsafe extern "C" fn el1_fiq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_FIQ);
}

#[no_mangle]
unsafe extern "C" fn el1_error(ctx: &mut ExceptionCtx) {
    catch(ctx, EL1_ERROR);
}

#[no_mangle]
unsafe extern "C" fn el0_sync(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_SYNC);
}

#[no_mangle]
unsafe extern "C" fn el0_irq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_IRQ);
}

#[no_mangle]
unsafe extern "C" fn el0_fiq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_FIQ);
}

#[no_mangle]
unsafe extern "C" fn el0_error(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_ERROR);
}

#[no_mangle]
unsafe extern "C" fn el0_32_sync(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_32_SYNC);
}

#[no_mangle]
unsafe extern "C" fn el0_32_irq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_32_IRQ);
}

#[no_mangle]
unsafe extern "C" fn el0_32_fiq(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_32_FIQ);
}

#[no_mangle]
unsafe extern "C" fn el0_32_error(ctx: &mut ExceptionCtx) {
    catch(ctx, EL0_32_ERROR);
}