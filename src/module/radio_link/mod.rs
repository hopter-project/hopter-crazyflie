use super::{
    acquire_irqsafe_nrf_usart,
    crtp::{SystemPacket, SYSTEM_MAX_DATA_SIZE},
};
use crate::module::crtp::SystemPacketGroupType;
use hopter::{
    interrupt::declare::{handler, irq},
    sync::Producer,
    time::sleep_ms,
};
use stm32f4xx_hal::prelude::*;

enum RadioRxState {
    Byte1,
    Byte2,
    Type,
    Size,
    Data,
    CRC1,
    CRC2,
}

pub struct RadioRxLink {
    sender: Option<Producer<SystemPacket, 16>>,
    state: RadioRxState,
    crc: [u8; 2],
    packet: SystemPacket,
    data_index: usize,
}

impl RadioRxLink {
    pub fn new() -> Self {
        Self {
            sender: None,
            state: RadioRxState::Byte1,
            crc: [0; 2],
            packet: SystemPacket::default(),
            data_index: 0,
        }
    }

    pub fn set_sender(&mut self, sender: Producer<SystemPacket, 16>) {
        self.sender = Some(sender);
    }

    fn dispatch_packet(&mut self, packet: SystemPacket) {
        if packet.type_ != 19
            && packet.type_ != 4
            && !(packet.data[0] >> 4 == 0xf && packet.data[0] & 0x3 == 0x03)
        {
            dbg_println!(
                "Get: {:?} {:?} 0x{:x}",
                packet.type_,
                packet.size,
                packet.data[0]
            );
        }
        match SystemPacketGroupType::from_u8(packet.type_) {
            SystemPacketGroupType::Radio => {
                if let Some(sender) = &self.sender {
                    let _ = sender.try_produce_allow_isr(packet);
                }
            }
            SystemPacketGroupType::PowerManagement => {}
            SystemPacketGroupType::OW => {}
            SystemPacketGroupType::System => {}
        }
    }

    pub fn usart_nrf_handle_data_isr(&mut self, b: u8) {
        match self.state {
            RadioRxState::Byte1 => {
                if b == 0xBC {
                    self.state = RadioRxState::Byte2;
                }
            }
            RadioRxState::Byte2 => {
                if b == 0xCF {
                    self.state = RadioRxState::Type;
                } else {
                    self.state = RadioRxState::Byte1;
                }
            }
            RadioRxState::Type => {
                self.crc[0] = b;
                self.crc[1] = b;
                self.packet.type_ = b;
                self.state = RadioRxState::Size;
            }
            RadioRxState::Size => {
                if b > 0 && b < 64 {
                    self.packet.size = b;
                    self.crc[0] += b;
                    self.crc[1] += self.crc[0];
                    self.data_index = 0;
                    self.state = RadioRxState::Data;
                } else {
                    self.state = RadioRxState::Byte1;
                }
            }
            RadioRxState::Data => {
                self.packet.data[self.data_index] = b;
                self.crc[0] += b;
                self.crc[1] += self.crc[0];
                self.data_index += 1;
                if self.data_index == self.packet.size as usize {
                    self.state = RadioRxState::CRC1;
                }
            }
            RadioRxState::CRC1 => {
                if self.crc[0] == b {
                    self.state = RadioRxState::CRC2;
                } else {
                    self.state = RadioRxState::Byte1;
                }
            }
            RadioRxState::CRC2 => {
                // only receive packets with type 0 which is RADIO_RAW packet
                if self.crc[1] == b {
                    self.dispatch_packet(self.packet.clone());
                }
                self.state = RadioRxState::Byte1;
            }
        }
    }
}

pub struct RadioTxLink {}

impl RadioTxLink {
    pub fn new() -> Self {
        Self {}
    }

    pub fn init(&mut self) {
        self.set_channel(80);
        sleep_ms(2).unwrap();
        self.set_data_rate(2);
        sleep_ms(2).unwrap();
        self.set_address(&[0xE7, 0xE7, 0xE7, 0xE7, 0xE7]);
    }

    fn set_channel(&mut self, channel: u8) {
        let mut packet = SystemPacket::default();
        packet.type_ = 1;
        packet.size = 1;
        packet.data[0] = channel;
        self.send_packet(packet);
    }

    fn set_data_rate(&mut self, data_rate: u8) {
        let mut packet = SystemPacket::default();
        packet.type_ = 2;
        packet.size = 1;
        packet.data[0] = data_rate;
        self.send_packet(packet);
    }

    fn set_address(&mut self, address: &[u8]) {
        let mut packet = SystemPacket::default();
        packet.type_ = 5;
        packet.size = 5;
        for i in 0..5 {
            packet.data[i] = address[i];
        }
        self.send_packet(packet);
    }

    pub fn send_packet(&mut self, packet: SystemPacket) {
        dbg_println!("send {}", packet.type_);
        let mut crc = [0_u8; 2];
        let mut buffer = [0_u8; SYSTEM_MAX_DATA_SIZE + 6];
        buffer[0] = 0xBC;
        buffer[1] = 0xCF;
        buffer[2] = packet.type_;
        buffer[3] = packet.size;

        let size = packet.size as usize;
        buffer[4..4 + size].copy_from_slice(&packet.data[0..size]);

        for i in 2..size + 4 {
            crc[0] += buffer[i];
            crc[1] += crc[0];
        }

        buffer[size + 4] = crc[0];
        buffer[size + 5] = crc[1];

        {
            let mut tx = acquire_irqsafe_nrf_usart();
            unsafe { tx.write_dma(&buffer[0..size + 6], None).unwrap() };
        }
        let sem = super::borrow_nrf_usart_sem();
        sem.down();
    }
}

irq!(Usart6Irq, stm32f4xx_hal::pac::Interrupt::USART6);
irq!(Dma2Stream7Irq, stm32f4xx_hal::pac::Interrupt::DMA2_STREAM7);
pub type NrfUartDmaIrq = (Usart6Irq, Dma2Stream7Irq);

/// Handle DMA interrupt. Increase the semaphore count so that the task waiting
/// for DMA read/write to finish can resume.
#[handler(DMA2_STREAM7)]
fn dma2_stream7_handler() {
    let sem = super::borrow_nrf_usart_sem();
    {
        let mut handle = super::acquire_irqsafe_nrf_usart();
        handle.handle_dma_interrupt();
    }

    let _ = sem.try_up_allow_isr();
}

#[handler(USART6)]
fn usart6_handler() {
    let mut radio = super::acquire_irqsafe_radio_rx_link();
    let mut usart = super::acquire_irqsafe_nrf_usart();
    let mut data = [0u8; 1];
    if let Ok(()) = usart.read(&mut data) {
        radio.usart_nrf_handle_data_isr(data[0]);
    }
}
