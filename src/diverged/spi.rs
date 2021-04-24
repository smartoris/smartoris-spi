use drone_cortexm::reg::prelude::*;
use drone_stm32_map::periph::spi::{SpiMap, SpiPeriph};

#[allow(dead_code)]
pub(crate) struct SpiDiverged<T: SpiMap> {
    pub(crate) rcc_busenr_spien: T::SRccBusenrSpien,
    pub(crate) rcc_busrstr_spirst: T::SRccBusrstrSpirst,
    pub(crate) rcc_bussmenr_spismen: T::SRccBussmenrSpismen,
    pub(crate) spi_cr1: T::USpiCr1,
    pub(crate) spi_cr2: T::USpiCr2,
    pub(crate) spi_sr: T::CSpiSr,
    pub(crate) spi_dr: T::USpiDr,
    pub(crate) spi_crcpr: T::USpiCrcpr,
    pub(crate) spi_rxcrcr: T::USpiRxcrcr,
    pub(crate) spi_txcrcr: T::USpiTxcrcr,
    pub(crate) spi_i2scfgr: T::SSpiI2ScfgrOpt,
    pub(crate) spi_i2spr: T::SSpiI2SprOpt,
}

impl<T: SpiMap> From<SpiPeriph<T>> for SpiDiverged<T> {
    fn from(periph: SpiPeriph<T>) -> Self {
        let SpiPeriph {
            rcc_busenr_spien,
            rcc_busrstr_spirst,
            rcc_bussmenr_spismen,
            spi_cr1,
            spi_cr2,
            spi_sr,
            spi_dr,
            spi_crcpr,
            spi_rxcrcr,
            spi_txcrcr,
            spi_i2scfgr,
            spi_i2spr,
        } = periph;
        Self {
            rcc_busenr_spien,
            rcc_busrstr_spirst,
            rcc_bussmenr_spismen,
            spi_cr1: spi_cr1.into_unsync(),
            spi_cr2: spi_cr2.into_unsync(),
            spi_sr: spi_sr.into_copy(),
            spi_dr: spi_dr.into_unsync(),
            spi_crcpr: spi_crcpr.into_unsync(),
            spi_rxcrcr: spi_rxcrcr.into_unsync(),
            spi_txcrcr: spi_txcrcr.into_unsync(),
            spi_i2scfgr,
            spi_i2spr,
        }
    }
}
