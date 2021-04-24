use crate::SpiDrv;
use core::{mem::ManuallyDrop, slice::SliceIndex};
use drone_cortexm::thr::prelude::*;
use drone_stm32_map::periph::{dma::ch::DmaChMap, spi::SpiMap};

/// SPI master session.
///
/// The session object takes ownership of the provided buffer, which is returned
/// by [`SpiMaster::stop`] method. If the `stop` method is not called, the
/// buffer will be leaked.
pub struct SpiMaster<
    'a,
    Spi: SpiMap,
    SpiInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> {
    drv: &'a mut SpiDrv<Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt>,
    buf: ManuallyDrop<Box<[u8]>>,
}

impl<
    'a,
    Spi: SpiMap,
    SpiInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> SpiMaster<'a, Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt>
{
    pub(crate) fn new(
        drv: &'a mut SpiDrv<Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt>,
        buf: Box<[u8]>,
    ) -> Self {
        Self { drv, buf: ManuallyDrop::new(buf) }
    }

    /// Simultaneously transmits the contents of the session buffer slice of the
    /// range `index` to the slave, and receives the data from the slave to the
    /// same session buffer slice.
    pub async fn exchange<I: SliceIndex<[u8], Output = [u8]>>(
        mut self,
        index: I,
    ) -> SpiMaster<'a, Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        unsafe { self.drv.exchange(&mut self.buf[index]).await };
        self
    }

    /// Returns a reference to the session buffer.
    #[inline]
    #[must_use]
    pub fn buf(&self) -> &[u8] {
        &self.buf
    }

    /// Returns a mutable reference to the session buffer.
    #[inline]
    #[must_use]
    pub fn buf_mut(&mut self) -> &mut Box<[u8]> {
        &mut self.buf
    }

    /// Drops the current session and returns the session buffer.
    #[inline]
    #[must_use]
    pub fn stop(self) -> Box<[u8]> {
        let Self { buf, .. } = self;
        ManuallyDrop::into_inner(buf)
    }
}
