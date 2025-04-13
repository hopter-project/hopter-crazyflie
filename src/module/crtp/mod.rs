use super::clone_radio_tx_producer;

pub const CRTP_MAX_DATA_SIZE: usize = 31;
pub const SYSTEM_MAX_DATA_SIZE: usize = 64;

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum CRTPPort {
    Console = 0x0,
    Param = 0x2,
    Setpoint = 0x3,
    Mem = 0x4,
    Log = 0x5,
    Localization = 0x6,
    SetpointGeneric = 0x7,
    SetpointHl = 0x8,
    Platform = 0xD,
    Link = 0xF,
}

pub enum SystemPacketGroupType {
    Radio = 0x00,
    PowerManagement = 0x10,
    OW = 0x20,
    System = 0x30,
}

impl SystemPacketGroupType {
    pub fn from_u8(value: u8) -> Self {
        match value & 0xF0 {
            0x00 => Self::Radio,
            0x10 => Self::PowerManagement,
            0x20 => Self::OW,
            0x30 => Self::System,
            _ => panic!("Invalid SystemPacketGroupType"),
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct CRTPPacket {
    pub size: u8,
    pub data_raw: [u8; CRTP_MAX_DATA_SIZE],
}

#[derive(Copy, Clone, Debug)]
pub struct SystemPacket {
    pub type_: u8,
    pub size: u8,
    pub data: [u8; SYSTEM_MAX_DATA_SIZE],
}

impl Default for SystemPacket {
    fn default() -> Self {
        Self {
            type_: 0,
            size: 0,
            data: [0; SYSTEM_MAX_DATA_SIZE],
        }
    }
}

impl SystemPacket {
    pub fn reset(&mut self) {
        self.type_ = 0;
        self.size = 0;
        self.data = [0; SYSTEM_MAX_DATA_SIZE];
    }

    pub fn from_crtp_packet(packet: CRTPPacket) -> Self {
        let mut system_packet = Self::default();
        system_packet.type_ = 0;
        system_packet.size = packet.size + 1;
        system_packet.data[..packet.size as usize + 1]
            .copy_from_slice(&packet.data_raw[..packet.size as usize + 1]);
        system_packet
    }

    pub fn send(&self) {
        let prod = clone_radio_tx_producer();
        let _ = prod.try_produce_allow_isr(self.clone());
    }
}

impl CRTPPacket {
    pub fn new(port: CRTPPort, channel: u8, data: &[u8]) -> Self {
        let mut packet = Self::default();
        packet.size = data.len() as u8;
        packet.data_raw[0] = (port as u8 & 0x0F) << 4 | (channel & 0x0F);
        packet.data_raw[1..data.len() + 1].copy_from_slice(data);
        packet
    }

    pub fn channel(&self) -> u8 {
        self.data_raw[0] & 0x03
    }

    pub fn port(&self) -> CRTPPort {
        match self.data_raw[0] >> 4 {
            0x0 => CRTPPort::Console,
            0x2 => CRTPPort::Param,
            0x3 => CRTPPort::Setpoint,
            0x4 => CRTPPort::Mem,
            0x5 => CRTPPort::Log,
            0x6 => CRTPPort::Localization,
            0x7 => CRTPPort::SetpointGeneric,
            0x8 => CRTPPort::SetpointHl,
            0xD => CRTPPort::Platform,
            0xF => CRTPPort::Link,
            _ => panic!("Invalid CRTP port"),
        }
    }

    pub fn set_data(&mut self, data: &[u8]) {
        self.size = data.len() as u8;
        self.data_raw[1..data.len() + 1].copy_from_slice(data);
    }

    pub fn reset(&mut self) {
        self.size = 0;
        self.data_raw = [0; CRTP_MAX_DATA_SIZE];
    }

    pub fn send(&self) {
        let prod = clone_radio_tx_producer();
        let _ = prod.try_produce_allow_isr(SystemPacket::from_crtp_packet(self.clone()));
    }
}

impl From<SystemPacket> for CRTPPacket {
    fn from(packet: SystemPacket) -> Self {
        let mut crtp_packet = Self::default();
        crtp_packet.size = packet.size - 1;
        crtp_packet.data_raw[..crtp_packet.size as usize]
            .copy_from_slice(&packet.data[..crtp_packet.size as usize]);
        crtp_packet
    }
}
