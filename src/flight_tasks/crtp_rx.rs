use crate::module::{
    self,
    controller::types::SetPointT,
    crtp::{CRTPPacket, CRTPPort, SystemPacket},
};
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use hopter::{
    sync::{Consumer, Producer},
    task,
    time::{self, IntervalBarrier},
};

const FLY: bool = false;

pub fn build_crtp_rx_task(
    cons_system_packet: Consumer<SystemPacket, 16>,
    prod_setpoint: Producer<SetPointT, 2>,
) {
    task::build()
        .set_id(super::TASK_ID_CRTP_RX)
        .set_priority(super::TASK_PRIO_CRTP_RX)
        .set_entry(|| crtp_rx_task(cons_system_packet, prod_setpoint))
        .set_stack_init_size(super::TASK_STK_SIZE_CRTP_RX)
        .spawn_restartable()
        .unwrap();
}

static COUNT: AtomicUsize = AtomicUsize::new(0);

fn crtp_rx_task(
    cons_system_packet: Consumer<SystemPacket, 16>,
    prod_setpoint: Producer<SetPointT, 2>,
) {
    static HAS_STARTED: AtomicBool = AtomicBool::new(false);
    if !HAS_STARTED.load(Ordering::SeqCst) {
        module::system_wait_start();
        HAS_STARTED.store(true, Ordering::SeqCst);
        dbg_println!("CRTP-RX [START]");
        time::sleep_ms(4000).unwrap();
    }

    let mut barrier = IntervalBarrier::new(100).unwrap();

    super::enable_watch_dog();

    loop {
        barrier.wait();

        super::crtp_rx_report_alive();

        let count = COUNT.fetch_add(1, Ordering::Relaxed);

        #[cfg(feature = "crtp_rx_panic")]
        {
            if (count + 1) % 10 == 0 {
                panic!()
            }
        }

        // ####################################################################################
        // # Hardcoded commands for hovering the drone at 0.5m for 10 seconds after take-off. #
        // ####################################################################################

        if FLY {
            if count < 10 {
                let mut setpoint = SetPointT::default();
                setpoint.thrust = 47000.0;
                prod_setpoint.produce(setpoint);
                continue;
            } else if count < 100 {
                let mut setpoint = SetPointT::default_hover_packet();
                // hover at 0.5m
                setpoint.position.z = 0.5;
                prod_setpoint.produce(setpoint);
                continue;
            }
        }

        // #############################################################################
        // # Code below are currently unused.                                          #
        // # They are some initial draft for handling control packets sent over radio. #
        // #############################################################################

        let packet = cons_system_packet.consume();
        // SYSLINK_RADIO_RAW type
        if packet.type_ == 0x00 {
            // debug::debug_print(&format!("Get radio: {:?}\n", packet.type_));
            let mut crtp_packet: CRTPPacket = packet.into();
            match crtp_packet.port() {
                CRTPPort::Link => {
                    if crtp_packet.channel() == 1 {
                        // ‘Bitcrazy Crazyflie’
                        let link_data = &[
                            0x42, 0x69, 0x74, 0x63, 0x72, 0x61, 0x7a, 0x65, 0x20, 0x43, 0x72, 0x61,
                            0x7a, 0x79, 0x66, 0x6c, 0x69, 0x65,
                        ];
                        crtp_packet.set_data(link_data);
                        crtp_packet.send();
                    }
                }
                CRTPPort::Platform => {
                    if crtp_packet.channel() == 1 {
                        // protocol version: 4
                        crtp_packet.set_data(&[0x00, 0x04]);
                        crtp_packet.send();
                    }
                }
                CRTPPort::Param => {
                    if crtp_packet.channel() == 0 {
                        if crtp_packet.data_raw[1] == 3 {
                            crtp_packet.set_data(&[0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
                            crtp_packet.send();
                        }
                    }
                }
                CRTPPort::Log => {
                    dbg_println!(
                        "Get log: {:?} {}",
                        crtp_packet.channel(),
                        crtp_packet.data_raw[1]
                    );
                    if crtp_packet.channel() == 1 {
                        if crtp_packet.data_raw[1] == 5 {
                            crtp_packet.set_data(&[0x05, 0x00, 0x00]);
                            crtp_packet.send();
                        }
                    }
                }
                CRTPPort::Mem => {
                    if crtp_packet.channel() == 0 {
                        if crtp_packet.data_raw[1] == 1 {
                            crtp_packet.set_data(&[0x01, 0x00]);
                            crtp_packet.send();
                        }
                    }
                }
                CRTPPort::Setpoint => {}
                CRTPPort::SetpointGeneric => {}
                _ => {}
            }
        } else {
            if packet.type_ != 4 && packet.type_ != 19 {
                dbg_println!("Get non-radio: {:?}", packet.type_);
            }
        }
    }
}
