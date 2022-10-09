// RTIC Monotonic impl for the RTCs
// https://gist.github.com/korken89/fe94a475726414dd1bce031c76adc3dd
// Modified to use 32bit values for nrf51
// Removed RTC2 (not implemented on nrf51)

use crate::hal::pac::{rtc0, RTC0, RTC1};
pub use fugit::{self, ExtU32};
use rtic_monotonic::Monotonic;

pub struct MonoRtc<T: InstanceRtc> {
    overflow: u8,
    rtc: T,
}

impl<T: InstanceRtc> MonoRtc<T> {
    pub fn new(rtc: T) -> Self {
        unsafe { rtc.prescaler.write(|w| w.bits(0)) };

        MonoRtc { overflow: 0, rtc }
    }

    pub fn is_overflow(&self) -> bool {
        self.rtc.events_ovrflw.read().bits() == 1
    }
}

impl<T: InstanceRtc> Monotonic for MonoRtc<T> {
    type Instant = fugit::TimerInstantU32<32_768>;
    type Duration = fugit::TimerDurationU32<32_768>;
    
    const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

    unsafe fn reset(&mut self) {
        self.rtc
            .intenset
            .write(|w| w.compare0().set().ovrflw().set());
        self.rtc
            .evtenset
            .write(|w| w.compare0().set().ovrflw().set());

        self.rtc.tasks_clear.write(|w| w.bits(1));
        self.rtc.tasks_start.write(|w| w.bits(1));
    }

    #[inline(always)]
    fn now(&mut self) -> Self::Instant {
        let cnt = self.rtc.counter.read().bits();
        let ovf = if self.is_overflow() {
            self.overflow.wrapping_add(1)
        } else {
            self.overflow
        } as u32;

        Self::Instant::from_ticks((ovf << 24) | cnt)
    }

    fn set_compare(&mut self, instant: Self::Instant) {
        let now = self.now();
        
        const MIN_TICKS_FOR_COMPARE: u32 = 3;

        // Since the timer may or may not overflow based on the requested compare val, we check
        // how many ticks are left.
        //
        // Note: The RTC cannot have a compare value too close to the current timer value,
        // so we use the `MIN_TICKS_FOR_COMPARE` to set a minimum offset from now to the set value.
        let val = match instant.checked_duration_since(now) {
            Some(x) if x.ticks() <= 0xffffff && x.ticks() > MIN_TICKS_FOR_COMPARE => {
                instant.duration_since_epoch().ticks() & 0xffffff
            } // Will not overflow
            Some(x) => {
                (instant.duration_since_epoch().ticks() + (MIN_TICKS_FOR_COMPARE - x.ticks()))
                    & 0xffffff
            } // Will not overflow
            _ => 0, // Will overflow or in the past, set the same value as after overflow to not get extra interrupts
        };

        unsafe { self.rtc.cc[0].write(|w| w.bits(val)) };
    }

    fn clear_compare_flag(&mut self) {
        unsafe { self.rtc.events_compare[0].write(|w| w.bits(0)) };
    }

    #[inline(always)]
    fn zero() -> Self::Instant {
        Self::Instant::from_ticks(0)
    }

    fn on_interrupt(&mut self) {
        if self.is_overflow() {
            self.overflow = self.overflow.wrapping_add(1);
            self.rtc.events_ovrflw.write(|w| unsafe { w.bits(0) });
        }
    }
}

pub trait InstanceRtc: core::ops::Deref<Target = rtc0::RegisterBlock> {}
impl InstanceRtc for RTC0 {}
impl InstanceRtc for RTC1 {}
