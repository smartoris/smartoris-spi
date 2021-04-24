//! SPI [Drone OS] driver for STM32F4 micro-controllers.
//!
//! # Limitations
//!
//! * Transmission and reception works only through DMA channels with
//! interrupts. Polling and interrupt only methods are not supported.
//!
//! * Errors from peripherals are handled via panicking.
//!
//! * Only the master role is implemented.
//!
//! * Only 8-bit data frame format is implemented.
//!
//! * 1-line bidirectional data mode is not supported.
//!
//! # Usage
//!
//! Add the crate to your `Cargo.toml` dependencies:
//!
//! ```toml
//! [dependencies]
//! smartoris-spi = { version = "0.1.0" }
//! ```
//!
//! Add or extend `std` feature as follows:
//!
//! ```toml
//! [features]
//! std = ["smartoris-spi/std"]
//! ```
//!
//! Example of initializing the driver for SPI1, DMA2 CH0/CH3, and A5/A6/A7
//! pins:
//!
//! ```no_run
//! # #![feature(const_fn_fn_ptr_basics)]
//! # use drone_stm32_map::stm32_reg_tokens;
//! # use drone_core::token::Token;
//! # stm32_reg_tokens! {
//! #     index => Regs;
//! #     exclude => {
//! #         scb_ccr,
//! #         mpu_type, mpu_ctrl, mpu_rnr, mpu_rbar, mpu_rasr,
//! #     }
//! # }
//! mod thr {
//!     pub use drone_cortexm::thr::init;
//!     pub use drone_stm32_map::thr::*;
//!
//!     use drone_cortexm::thr;
//!
//!     thr::nvic! {
//!         thread => pub Thr {};
//!         local => pub ThrLocal {};
//!         vtable => pub Vtable;
//!         index => pub Thrs;
//!         init => pub ThrsInit;
//!
//!         threads => {
//!             interrupts => {
//!                 /// SPI1 global interrupt.
//!                 35: pub spi1;
//!                 /// DMA2 Stream2 global interrupt.
//!                 56: pub dma2_ch0;
//!                 /// DMA2 Stream3 global interrupt.
//!                 59: pub dma2_ch3;
//!             };
//!         };
//!     }
//! }
//!
//! use crate::thr::ThrsInit;
//! use drone_cortexm::{reg::prelude::*, thr::prelude::*};
//! use drone_stm32_map::periph::{
//!     dma::{periph_dma2, periph_dma2_ch0, periph_dma2_ch3},
//!     gpio::periph_gpio_a,
//!     spi::periph_spi1,
//! };
//! use smartoris_spi::{SpiDrv, SpiMode, SpiSetup};
//!
//! fn handler(reg: Regs, thr_init: ThrsInit) {
//!     let thr = thr::init(thr_init);
//!
//!     // Enable interrupts.
//!     thr.dma2_ch0.enable_int();
//!     thr.dma2_ch3.enable_int();
//!     thr.spi1.enable_int();
//!
//!     // Configure GPIO pins.
//!     let gpio_a = periph_gpio_a!(reg);
//!     gpio_a.rcc_busenr_gpioen.set_bit(); // IO port clock enable
//!     gpio_a.gpio_otyper.store(|r| {
//!         r.clear_ot5() // output push-pull
//!             .clear_ot7() // output push-pull
//!     });
//!     gpio_a.gpio_ospeedr.store(|r| {
//!         r.write_ospeedr5(0b11) // very high speed
//!             .write_ospeedr7(0b11) // very high speed
//!     });
//!     gpio_a.gpio_pupdr.store(|r| {
//!         r.write_pupdr5(0b00) // no pull-up, pull-down
//!             .write_pupdr6(0b00) // no pull-up, pull-down
//!             .write_pupdr7(0b00) // no pull-up, pull-down
//!     });
//!     gpio_a.gpio_afrl.store(|r| {
//!         r.write_afrl5(0b0101) // SPI1/SPI2/I2S2/SPI3/I2S3/SPI4
//!             .write_afrl6(0b0101) // SPI1/SPI2/I2S2/SPI3/I2S3/SPI4
//!             .write_afrl7(0b0101) // SPI1/SPI2/I2S2/SPI3/I2S3/SPI4
//!     });
//!     gpio_a.gpio_moder.store(|r| {
//!         r.write_moder5(0b10) // alternate function
//!             .write_moder6(0b10) // alternate function
//!             .write_moder7(0b10) // alternate function
//!     });
//!     gpio_a.rcc_busenr_gpioen.clear_bit(); // IO port clock disable
//!
//!     periph_dma2!(reg).rcc_busenr_dmaen.set_bit(); // DMA clock enable
//!
//!     // Set up the driver.
//!     let spi1 = SpiDrv::init(SpiSetup {
//!         spi: periph_spi1!(reg),
//!         spi_int: thr.spi1,
//!         spi_baud_rate: 0b000,             // f_plck/2
//!         spi_mode: SpiMode::LowIdleRising, // CPOL = 0, CPHA = 0
//!         spi_lsb_first: false,             // MSB first
//!         dma_tx: periph_dma2_ch3!(reg),
//!         dma_tx_int: thr.dma2_ch3,
//!         dma_tx_ch: 3,    // SPI1_TX
//!         dma_tx_pl: 0b11, // very high
//!         dma_rx: periph_dma2_ch0!(reg),
//!         dma_rx_int: thr.dma2_ch0,
//!         dma_rx_ch: 3,    // SPI1_RX
//!         dma_rx_pl: 0b11, // very high
//!     });
//! }
//! # fn main() {
//! #     unsafe { handler(Regs::take(), ThrsInit::take()) };
//! # }
//! ```
//!
//! Example of usage:
//!
//! ```no_run
//! # #![feature(const_fn_fn_ptr_basics)]
//! # use drone_stm32_map::periph::{
//! #     dma::ch::{Dma2Ch0, Dma2Ch3},
//! #     spi::Spi1,
//! # };
//! # mod thr {
//! #     use drone_stm32_map::thr::*;
//! #     drone_cortexm::thr::nvic! {
//! #         thread => pub Thr {};
//! #         local => pub ThrLocal {};
//! #         vtable => pub Vtable;
//! #         index => pub Thrs;
//! #         init => pub ThrsInit;
//! #         threads => {
//! #             interrupts => {
//! #                 35: pub spi1;
//! #                 56: pub dma2_ch0;
//! #                 59: pub dma2_ch3;
//! #             };
//! #         };
//! #     }
//! # }
//! # async fn handler() {
//! # let mut spi1: smartoris_spi::SpiDrv<
//! #     Spi1,
//! #     thr::Spi1,
//! #     Dma2Ch3,
//! #     thr::Dma2Ch3,
//! #     Dma2Ch0,
//! #     thr::Dma2Ch0,
//! # > = unsafe { core::mem::MaybeUninit::uninit().assume_init() };
//! let buf = vec![0x00, 0x39, 0x00, 0x00].into_boxed_slice();
//! let buf = spi1
//!     .master(buf) // create a master session backed by the given buffer
//!     .exchange(..) // transmit from and receive to all 4 bytes of the buffer
//!     .await
//!     .stop(); // get the buffer back
//! println!("{:?}", buf);
//! # }
//! # fn main() {}
//! ```
//!
//! # References
//!
//! [Drone OS]: https://www.drone-os.com/

#![feature(never_type)]
#![feature(prelude_import)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::module_name_repetitions,
    clippy::similar_names,
    clippy::wildcard_imports
)]
#![cfg_attr(not(feature = "std"), no_std)]

mod diverged;
mod drv;
mod master;

pub use self::{
    drv::{SpiDrv, SpiMode, SpiSetup},
    master::SpiMaster,
};

#[prelude_import]
#[allow(unused_imports)]
use drone_core::prelude::*;
