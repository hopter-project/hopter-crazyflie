use hopter::{
    interrupt::mask::MaskableIrq,
    sync::{MutexIrqSafeGuard, Semaphore},
    task,
};
use owning_ref::OwningRefMut;
use stm32f4xx_hal::{
    dma::{
        traits::{Channel, DMASet, Stream},
        ChannelX, MemoryToPeripheral, PeripheralToMemory, Stream0, Stream2, Stream4, Stream6,
    },
    i2c::{
        dma::{I2CMasterDma, I2CMasterWriteReadDMA, Rx, RxDMA, Tx, TxDMA},
        Instance,
    },
    pac::{DMA1, I2C1, I2C3},
};

pub type I2c3DMAHandle =
    I2CMasterDma<I2C3, TxDMA<I2C3, Stream4<DMA1>, 3>, RxDMA<I2C3, Stream2<DMA1>, 3>>;

pub type I2c1DMAHandle =
    I2CMasterDma<I2C1, TxDMA<I2C1, Stream6<DMA1>, 1>, RxDMA<I2C1, Stream0<DMA1>, 1>>;

pub fn i2c_dma_read<F, I, G, I2C, TxStream, const TX_CH: u8, RxStream, const RX_CH: u8>(
    acquire_dma_handle: F,
    get_sem: G,
    reg_addr: &[u8],
    reg_data: &mut [u8],
    intf: u32,
) -> i8
where
    F: Fn() -> OwningRefMut<
        'static,
        MutexIrqSafeGuard<
            'static,
            Option<I2CMasterDma<I2C, TxDMA<I2C, TxStream, TX_CH>, RxDMA<I2C, RxStream, RX_CH>>>,
            I,
        >,
        I2CMasterDma<I2C, TxDMA<I2C, TxStream, TX_CH>, RxDMA<I2C, RxStream, RX_CH>>,
    >,
    G: FnOnce() -> &'static Semaphore,
    I2C: Instance + 'static,
    TxStream: Stream + 'static,
    ChannelX<TX_CH>: Channel,
    Tx<I2C>: DMASet<TxStream, TX_CH, MemoryToPeripheral>,
    RxStream: Stream + 'static,
    ChannelX<RX_CH>: Channel,
    Rx<I2C>: DMASet<RxStream, RX_CH, PeripheralToMemory>,
    I: MaskableIrq + 'static,
{
    let mut locked_dma_handle = acquire_dma_handle();

    #[cfg(feature = "tof_panic")]
    {
        use crate::flight_tasks::TOF_PANIC_REQUIRED;
        use core::sync::atomic::Ordering;
        if TOF_PANIC_REQUIRED.load(Ordering::Relaxed) && TX_CH == 1 {
            panic!()
        }
    }

    #[cfg(feature = "accel_gyro_panic")]
    {
        use crate::flight_tasks::ACCEL_GYRO_PANIC_REQUIRED;
        use core::sync::atomic::Ordering;
        if ACCEL_GYRO_PANIC_REQUIRED.load(Ordering::Relaxed) && TX_CH == 3 {
            panic!()
        }
    }

    while locked_dma_handle.busy() {
        core::mem::drop(locked_dma_handle);
        task::yield_current();
        locked_dma_handle = acquire_dma_handle();
    }
    if let Err(_) =
        unsafe { locked_dma_handle.write_read_dma(intf as u8, reg_addr, reg_data, None) }
    {
        return -1;
    }
    core::mem::drop(locked_dma_handle);

    (get_sem)().down();
    return 0;
}

pub fn i2c_write<F, I, I2C, TxStream, const TX_CH: u8, RxStream, const RX_CH: u8>(
    acquire_dma_handle: F,
    reg_addr: &[u8],
    reg_data: &[u8],
    intf: u32,
) -> i8
where
    F: Fn() -> OwningRefMut<
        'static,
        MutexIrqSafeGuard<
            'static,
            Option<I2CMasterDma<I2C, TxDMA<I2C, TxStream, TX_CH>, RxDMA<I2C, RxStream, RX_CH>>>,
            I,
        >,
        I2CMasterDma<I2C, TxDMA<I2C, TxStream, TX_CH>, RxDMA<I2C, RxStream, RX_CH>>,
    >,
    I2C: Instance + 'static,
    TxStream: Stream + 'static,
    ChannelX<TX_CH>: Channel,
    Tx<I2C>: DMASet<TxStream, TX_CH, MemoryToPeripheral>,
    RxStream: Stream + 'static,
    ChannelX<RX_CH>: Channel,
    Rx<I2C>: DMASet<RxStream, RX_CH, PeripheralToMemory>,
    I: MaskableIrq + 'static,
{
    let mut locked_dma_handle = acquire_dma_handle();
    while locked_dma_handle.busy() {
        core::mem::drop(locked_dma_handle);
        task::yield_current();
        locked_dma_handle = acquire_dma_handle();
    }
    if let Err(_) = locked_dma_handle.write(intf as u8, &[reg_addr, reg_data].concat()) {
        return -1;
    }
    return 0;
}
