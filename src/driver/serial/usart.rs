use stm32f4xx_hal::{
    dma::{Stream3, Stream7},
    pac::{DMA1, DMA2, USART3, USART6},
    serial::{dma::NoDMA, dma::SerialDma as SerialDMA, dma::TxDMA},
};

pub type DebugSerialDMAHandle = SerialDMA<USART3, TxDMA<USART3, Stream3<DMA1>, 4>, NoDMA>;
pub type NRFSerialDMAHandle = SerialDMA<USART6, TxDMA<USART6, Stream7<DMA2>, 5>, NoDMA>;
