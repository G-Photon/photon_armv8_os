// 不使用标准库
#![no_std]
// 不使用预定义入口点
#![no_main]
#![allow(dead_code)]
#![feature(global_asm)]
#![feature(asm)]
#![feature(panic_info_message)]
#[macro_use]
mod uart_console;
mod pl061;
mod panic;
mod interrupts;
global_asm!(include_str!("start.s"));
#[no_mangle] // 不修改函数名
pub extern "C" fn not_main() {
//     let banner = r#"
//   ____   _____                          _____  __  __        ___                  _           _              
//  / __ \ / ____|                   /\   |  __ \|  \/  |      / _ \     ____       | |         | |             
// | |  | | (___     ___  _ __      /  \  | |__) | \  / |_   _| (_) |   / __ \ _ __ | |__   ___ | |_ ___  _ __  
// | |  | |\___ \   / _ \| '_ \    / /\ \ |  _  /| |\/| \ \ / /> _ <   / / _` | '_ \| '_ \ / _ \| __/ _ \| '_ \ 
// | |__| |____) | | (_) | | | |  / ____ \| | \ \| |  | |\ V /| (_) | | | (_| | |_) | | | | (_) | || (_) | | | |
//  \____/|_____/   \___/|_| |_| /_/    \_\_|  \_\_|  |_| \_/  \___/   \ \__,_| .__/|_| |_|\___/ \__\___/|_| |_|
//                                                                      \____/| |                               
//                                                                            |_|                               
// "#;
//     print_result!(print!("{}", banner));
    //println!("Hello, interrupt!");
    println!("It did not crash!");
    // loop {
    //     print!("-");
    // }
    interrupts::init_gicv2();
    loop {}
}
