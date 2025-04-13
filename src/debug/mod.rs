use cfg_if::cfg_if;

cfg_if! { if #[cfg(not(feature = "no_dbg_print"))] {

use crate::{module, flight_tasks};
use core::{
    fmt,
    sync::atomic::{AtomicBool, Ordering},
};
use heapless::String;
use stm32f4xx_hal::nb;
use stm32f4xx_hal::prelude::*;
use hopter::{interrupt::{declare::{handler, irq}}, task};

static DEBUG_READY: AtomicBool = AtomicBool::new(false);

pub fn set_debug_ready() {
    DEBUG_READY.store(true, Ordering::SeqCst);
}

fn is_debug_ready() -> bool {
    DEBUG_READY.load(Ordering::SeqCst)
}

pub fn build_debug_task() {
    task::build()
        .set_id(flight_tasks::TASK_ID_DEBUG_PRINTER)
        .set_entry(|| debug_printer_worker())
        .set_stack_init_size(2048)
        .set_priority(flight_tasks::TASK_PRIO_DEBUG_PRINTER)
        .spawn()
        .unwrap();
}

#[macro_export]
macro_rules! dbg_print {
    ($($arg:tt)*) => {
        crate::debug::debug_print_format_arg(format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! dbg_println {
    () => {
        dbg_print!("\n")
    };
    ($s:expr) => {
        dbg_print!(concat!($s, "\n"))
    };
    ($s:expr, $($tt:tt)*) => {
        dbg_print!(concat!($s, "\n"), $($tt)*)
    };
}

#[macro_export]
macro_rules! dbg_print_blocking {
    ($($arg:tt)*) => {
        crate::debug::debug_print_blocking_format_arg(format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! dbg_println_blocking {
    () => {
        dbg_print_blocking!("\n")
    };
    ($s:expr) => {
        dbg_print_blocking!(concat!($s, "\n"))
    };
    ($s:expr, $($tt:tt)*) => {
        dbg_print_blocking!(concat!($s, "\n"), $($tt)*)
    };
}

pub type DebugString = String<128>;

#[allow(unused)]
#[inline(never)]
pub fn debug_print_format_arg(args: fmt::Arguments) {
    let mut s = DebugString::new();
    let res = fmt::write(&mut s, args);
    if res.is_err() {
        s = DebugString::try_from("[AN OVERLENGTH STRING]").unwrap();
    }

    let prod = module::clone_debug_print_producer();
    let _ = prod.try_produce_allow_isr(s);
}

#[allow(unused)]
#[inline(never)]
pub fn debug_print_blocking_format_arg(args: fmt::Arguments) {
    if !is_debug_ready() {
        return;
    }

    let mut s = DebugString::new();
    let res = fmt::write(&mut s, args);
    if res.is_err() {
        s = DebugString::try_from("[AN OVERLENGTH STRING]").unwrap();
    }

    let mut handle = module::acquire_irqsafe_debug_usart_dma_handle();
    handle.write(s.as_bytes());
}

#[allow(unused)]
pub fn debug_printer_worker() -> ! {
    let cons = module::clone_debug_print_consumer();

    loop {
        let s = cons.consume();
        {
            let mut handle = module::acquire_irqsafe_debug_usart_dma_handle();

            // Just keep retrying until the UART is idle.
            unsafe {
                while let Err(nb::Error::WouldBlock) = handle.write_dma(s.as_bytes(), None) {}
            }
        }
        let sem = module::borrow_debug_usart_sem();
        sem.down();
    }
}

irq!(DebugUsartDmaIrq, stm32f4xx_hal::pac::Interrupt::DMA1_STREAM3);

#[handler(DMA1_STREAM3)]
fn dma1_stream3_handler() {
    let sem = module::borrow_debug_usart_sem();
    {
        let mut handle = module::acquire_irqsafe_debug_usart_dma_handle();
        handle.handle_dma_interrupt();
    }

    let _ = sem.try_up_allow_isr();
}

} else { // if NOT #[cfg(not(feature = "no_dbg_print"))]

#[macro_export]
macro_rules! dbg_print {
    ($($arg:expr),*) => { ($($arg),*) };
}

#[macro_export]
macro_rules! dbg_println {
    ($($arg:expr),*) => { ($($arg),*) };
}

#[macro_export]
macro_rules! dbg_print_blocking {
    ($($arg:expr),*) => { ($($arg),*) };
}

#[macro_export]
macro_rules! dbg_println_blocking {
    ($($arg:expr),*) => { ($($arg),*) };
}

}} // end if #[cfg(not(feature = "no_dbg_print"))]
