use embedded_hal::blocking::delay::DelayMs;
use fugit;
use asm_delay::{AsmDelay, bitrate::*};

use ds1302::Delay;

pub struct MyClock<const TIMER_HZ: u32> {
    asmd: AsmDelay,
}

impl<const TIMER_HZ: u32> MyClock<TIMER_HZ> {
    pub fn new() -> Self {
        Self { asmd: AsmDelay::new(1_u32.mhz()) }
    }
}

impl<const TIMER_HZ: u32> Delay<TIMER_HZ> for MyClock<TIMER_HZ> {
    type Error = core::convert::Infallible;

    fn now(&mut self) -> fugit::TimerInstantU32<TIMER_HZ> {
        fugit::TimerInstantU32::from_ticks(0)
    }

    fn start(&mut self, _duration: fugit::TimerDurationU32<TIMER_HZ>) -> Result<(), Self::Error> {
        Ok(())
    }

    fn wait(&mut self) -> nb::Result<(), Self::Error> {
        self.asmd.delay_ms(1_u32);
        Ok(())
    }
}