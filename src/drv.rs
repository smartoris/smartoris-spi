use crate::{
    diverged::{DmaChDiverged, SpiDiverged},
    SpiMaster,
};
use drone_cortexm::{fib, reg::prelude::*, thr::prelude::*};
use drone_stm32_map::periph::{
    dma::ch::{traits::*, DmaChMap, DmaChPeriph},
    spi::{traits::*, SpiMap, SpiPeriph},
};
use futures::prelude::*;

/// SPI setup.
pub struct SpiSetup<
    Spi: SpiMap,
    SpiInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> {
    /// SPI peripheral.
    pub spi: SpiPeriph<Spi>,
    /// SPI global interrupt.
    pub spi_int: SpiInt,
    /// SPI baud rate.
    ///
    /// This will be written to SPI_CR1.BR field. See the reference manual for
    /// details.
    pub spi_baud_rate: u32,
    /// SPI mode.
    ///
    /// This will be written to SPI_CR1.CPOL and SPI_CR1.CPHA fields. See the
    /// reference manual for details.
    pub spi_mode: SpiMode,
    /// Least significant bit first frame format.
    pub spi_lsb_first: bool,
    /// DMA Tx channel peripheral.
    pub dma_tx: DmaChPeriph<DmaTx>,
    /// DMA Tx channel interrupt.
    pub dma_tx_int: DmaTxInt,
    /// DMA Tx channel number.
    ///
    /// This will be written to DMA_SxCR.CHSEL field. See the reference manual
    /// for details.
    pub dma_tx_ch: u32,
    /// DMA Tx channel priority level.
    ///
    /// This will be written to DMA_SxCR.PL field. See the reference manual for
    /// details.
    pub dma_tx_pl: u32,
    /// DMA Rx channel peripheral.
    pub dma_rx: DmaChPeriph<DmaRx>,
    /// DMA Rx channel interrupt.
    pub dma_rx_int: DmaRxInt,
    /// DMA Rx channel number.
    ///
    /// This will be written to DMA_SxCR.CHSEL field. See the reference manual
    /// for details.
    pub dma_rx_ch: u32,
    /// DMA Rx channel priority level.
    ///
    /// This will be written to DMA_SxCR.PL field. See the reference manual for
    /// details.
    pub dma_rx_pl: u32,
}

/// SPI data capture clock edge.
#[derive(Clone, Copy)]
pub enum SpiMode {
    /// The clock pin has a low-level idle state. Capturing the rising
    /// edge. CPOL = 0, CPHA = 0.
    LowIdleRising,
    /// The clock pin has a low-level idle state. Capturing the falling
    /// edge. CPOL = 0, CPHA = 1.
    LowIdleFalling,
    /// The clock pin has a high-level idle state. Capturing the falling
    /// edge. CPOL = 1, CPHA = 0.
    HighIdleFalling,
    /// The clock pin has a high-level idle state. Capturing the rising
    /// edge. CPOL = 1, CPHA = 1.
    HighIdleRising,
}

/// SPI driver.
pub struct SpiDrv<
    Spi: SpiMap,
    SpiInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> {
    spi: SpiDiverged<Spi>,
    spi_int: SpiInt,
    dma_tx: DmaChDiverged<DmaTx>,
    dma_tx_int: DmaTxInt,
    dma_rx: DmaChDiverged<DmaRx>,
    dma_rx_int: DmaRxInt,
}

impl<
    Spi: SpiMap,
    SpiInt: IntToken,
    DmaTx: DmaChMap,
    DmaTxInt: IntToken,
    DmaRx: DmaChMap,
    DmaRxInt: IntToken,
> SpiDrv<Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt>
{
    /// Sets up a new [`SpiDrv`] from `setup` values.
    #[must_use]
    pub fn init(setup: SpiSetup<Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt>) -> Self {
        let SpiSetup {
            spi,
            spi_int,
            spi_baud_rate,
            spi_mode,
            spi_lsb_first,
            dma_tx,
            dma_tx_int,
            dma_tx_ch,
            dma_tx_pl,
            dma_rx,
            dma_rx_int,
            dma_rx_ch,
            dma_rx_pl,
        } = setup;
        let mut drv = Self {
            spi: spi.into(),
            spi_int,
            dma_tx: dma_tx.into(),
            dma_tx_int,
            dma_rx: dma_rx.into(),
            dma_rx_int,
        };
        drv.init_spi(spi_baud_rate, spi_mode, spi_lsb_first);
        drv.init_dma_tx(dma_tx_ch, dma_tx_pl);
        drv.init_dma_rx(dma_rx_ch, dma_rx_pl);
        drv
    }

    /// Creates a new master session.
    ///
    /// The returned session object takes ownership of `buf`, which can be later
    /// returned by [`SpiMaster::stop`] method. If the `stop` method is not
    /// called, `buf` is leaked.
    #[inline]
    pub fn master(
        &mut self,
        buf: Box<[u8]>,
    ) -> SpiMaster<'_, Spi, SpiInt, DmaTx, DmaTxInt, DmaRx, DmaRxInt> {
        SpiMaster::new(self, buf)
    }

    pub(crate) unsafe fn exchange(&mut self, buf: &mut [u8]) -> impl Future<Output = ()> {
        let dma_rx = self.dma_rx(buf);
        self.dma_tx(buf);
        dma_rx
    }

    unsafe fn dma_tx(&mut self, buf_tx: &[u8]) {
        self.dma_tx.dma_cm0ar.store_reg(|r, v| {
            r.m0a().write(v, buf_tx.as_ptr() as u32); // memory address
        });
        self.dma_tx.dma_cndtr.store_reg(|r, v| {
            r.ndt().write(v, buf_tx.len() as u32); // number of data items to transfer
        });
        self.dma_tx.dma_ifcr_ctcif.set_bit(); // clear transfer complete interrupt flag
        self.dma_tx.dma_ccr.modify_reg(|r, v| r.en().set(v)); // stream enable
    }

    unsafe fn dma_rx(&mut self, buf_rx: &mut [u8]) -> impl Future<Output = ()> {
        let dma_ifcr_ctcif = self.dma_rx.dma_ifcr_ctcif;
        let dma_isr_dmeif = self.dma_rx.dma_isr_dmeif;
        let dma_isr_feif = self.dma_rx.dma_isr_feif;
        let dma_isr_tcif = self.dma_rx.dma_isr_tcif;
        let dma_isr_teif = self.dma_rx.dma_isr_teif;
        let future = self.dma_rx_int.add_future(fib::new_fn(move || {
            let val = dma_isr_tcif.load_val();
            handle_dma_err::<DmaRx>(&val, dma_isr_dmeif, dma_isr_feif, dma_isr_teif);
            if dma_isr_tcif.read(&val) {
                // transfer complete interrupt flag
                dma_ifcr_ctcif.set_bit(); // clear transfer complete interrupt flag
                fib::Complete(())
            } else {
                fib::Yielded(())
            }
        }));
        self.dma_rx.dma_cm0ar.store_reg(|r, v| {
            r.m0a().write(v, buf_rx.as_mut_ptr() as u32); // memory address
        });
        self.dma_rx.dma_cndtr.store_reg(|r, v| {
            r.ndt().write(v, buf_rx.len() as u32); // number of data items to transfer
        });
        self.dma_rx.dma_ccr.modify_reg(|r, v| r.en().set(v)); // stream enable
        future
    }

    fn init_spi(&mut self, spi_baud_rate: u32, spi_mode: SpiMode, spi_lsb_first: bool) {
        self.spi.rcc_busenr_spien.set_bit(); // SPI clock enable
        self.spi.spi_cr2.store_reg(|r, v| {
            r.errie().set(v); // error interrupt is enabled
            r.frf().clear(v); // SPI Motorola mode
            r.txdmaen().set(v); // Tx buffer DMA enabled
            r.rxdmaen().set(v); // Rx buffer DMA enabled
        });
        self.spi.spi_cr1.store_reg(|r, v| {
            r.bidimode().clear(v); // 2-line unidirectional data mode selected
            r.dff().clear(v); // data frame format
            r.ssm().set(v); // software slave management enabled
            r.ssi().set(v); // internal slave select
            if spi_lsb_first {
                r.lsbfirst().set(v); // frame format
            } else {
                r.lsbfirst().clear(v); // frame format
            }
            r.spe().set(v); // peripheral enabled
            r.br().write(v, spi_baud_rate); // baud rate control
            r.mstr().set(v); // master configuration
            match spi_mode {
                SpiMode::LowIdleRising => {
                    r.cpol().clear(v); // clock polarity
                    r.cpha().clear(v); // clock phase
                }
                SpiMode::LowIdleFalling => {
                    r.cpol().clear(v); // clock polarity
                    r.cpha().set(v); // clock phase
                }
                SpiMode::HighIdleFalling => {
                    r.cpol().set(v); // clock polarity
                    r.cpha().clear(v); // clock phase
                }
                SpiMode::HighIdleRising => {
                    r.cpol().set(v); // clock polarity
                    r.cpha().set(v); // clock phase
                }
            }
        });
        let spi_sr = self.spi.spi_sr;
        self.spi_int.add_fn(move || {
            let val = spi_sr.load_val();
            handle_spi_err::<Spi>(&val, spi_sr);
            fib::Yielded::<(), !>(())
        });
    }

    fn init_dma_tx(&mut self, channel: u32, priority: u32) {
        let address = self.spi.spi_dr.as_mut_ptr(); // SPI data register
        self.dma_tx.dma_cpar.store_reg(|r, v| {
            r.pa().write(v, address as u32); // peripheral address
        });
        self.dma_tx.dma_ccr.store_reg(|r, v| {
            r.chsel().write(v, channel); // channel selection
            r.pl().write(v, priority); // priority level
            r.msize().write(v, 0b00); // byte (8-bit)
            r.psize().write(v, 0b00); // byte (8-bit)
            r.minc().set(v); // memory address pointer is incremented after each data transfer
            r.pinc().clear(v); // peripheral address pointer is fixed
            r.dir().write(v, 0b01); // memory-to-peripheral
            r.tcie().clear(v); // transfer complete interrupt disable
            r.teie().set(v); // transfer error interrupt enable
        });
        let dma_isr_dmeif = self.dma_tx.dma_isr_dmeif;
        let dma_isr_feif = self.dma_tx.dma_isr_feif;
        let dma_isr_teif = self.dma_tx.dma_isr_teif;
        self.dma_tx_int.add_fn(move || {
            let val = dma_isr_teif.load_val();
            handle_dma_err::<DmaTx>(&val, dma_isr_dmeif, dma_isr_feif, dma_isr_teif);
            fib::Yielded::<(), !>(())
        });
    }

    fn init_dma_rx(&mut self, channel: u32, priority: u32) {
        let address = self.spi.spi_dr.as_ptr(); // SPI data register
        self.dma_rx.dma_cpar.store_reg(|r, v| {
            r.pa().write(v, address as u32); // peripheral address
        });
        self.dma_rx.dma_ccr.store_reg(|r, v| {
            r.chsel().write(v, channel); // channel selection
            r.pl().write(v, priority); // priority level
            r.msize().write(v, 0b00); // byte (8-bit)
            r.psize().write(v, 0b00); // byte (8-bit)
            r.minc().set(v); // memory address pointer is incremented after each data transfer
            r.pinc().clear(v); // peripheral address pointer is fixed
            r.dir().write(v, 0b00); // peripheral-to-memory
            r.tcie().set(v); // transfer complete interrupt enable
            r.teie().set(v); // transfer error interrupt enable
        });
    }
}

fn handle_dma_err<T: DmaChMap>(
    val: &T::DmaIsrVal,
    dma_isr_dmeif: T::CDmaIsrDmeif,
    dma_isr_feif: T::CDmaIsrFeif,
    dma_isr_teif: T::CDmaIsrTeif,
) {
    if dma_isr_teif.read(&val) {
        panic!("Transfer error");
    }
    if dma_isr_dmeif.read(&val) {
        panic!("Direct mode error");
    }
    if dma_isr_feif.read(&val) {
        panic!("FIFO error");
    }
}

fn handle_spi_err<T: SpiMap>(val: &T::SpiSrVal, spi_sr: T::CSpiSr) {
    if spi_sr.modf().read(&val) {
        panic!("Master mode fault");
    }
    if spi_sr.ovr().read(&val) {
        panic!("Overrun condition");
    }
    if spi_sr.crcerr().read(&val) {
        panic!("CRC error");
    }
    if spi_sr.fre().read(&val) {
        panic!("TI frame format error");
    }
}
