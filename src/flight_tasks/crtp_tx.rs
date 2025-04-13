use crate::module;
use core::sync::atomic::{AtomicBool, Ordering};

#[allow(unused)]
pub fn crtp_tx_task(arg: ()) {
    static HAS_STARTED: AtomicBool = AtomicBool::new(false);
    if !HAS_STARTED.load(Ordering::SeqCst) {
        module::system_wait_start();
        HAS_STARTED.store(true, Ordering::SeqCst)
    }
    dbg_println!("CRTP-TX [START]");
    {
        let mut tx = module::acquire_radio_tx_link();
        tx.init();
    }

    let cons = module::clone_radio_tx_consumer();
    loop {
        let s = cons.consume();
        {
            let mut tx = module::acquire_radio_tx_link();
            tx.send_packet(s);
        }
    }
}
