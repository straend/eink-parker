#![no_std]
#![no_main]

use panic_rtt_target as _;
use rtt_target::{
    rtt_init_print, 
    rprintln,
};

use nrf51_hal as hal;

mod reverser;
use reverser::Reverser;

mod monotonic_nrf51_rtc;
use monotonic_nrf51_rtc::MonoRtc;

mod myclock;
use myclock::MyClock;

use embedded_hal::{
    digital::v2::InputPin,
};
use rtic::app;
use shared_bus_rtic::CommonBus;

use fugit::{self, ExtU32};
use core::fmt::Write;

use hal::{
    gpio::{
        PushPull, Output, Input, Floating, Pin, PullUp, Level,
        p0::{self, P0_04, P0_17, P0_18, P0_23, P0_24},
    },
    gpiote::*,
    pac::{SPI1, TWI0, TIMER0, RTC1},
    spi,
    timer::Timer,
    twi::{self, Twi},
    rng::Rng,
    rtc::{Rtc, RtcCompareReg, RtcInterrupt},
};
use embedded_graphics::{
    prelude::*,
    text::TextStyle,
};
use eg_seven_segment::SevenSegmentStyle;

use epd_waveshare:: {
    epd2in7bc::{Epd2in7bc, Display2in7bc},
    graphics::TriDisplay,
    prelude::*,
};

use ds1302::{DS1302, Clock, Hours, Mode as ds1302_mode};

use ssd1306::{
    prelude::I2CInterface,
    Ssd1306, size::DisplaySize128x32, mode::TerminalMode,
    mode::DisplayConfig,
};


type SpiBus = spi::Spi<SPI1>;



#[repr(u8)]
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum MyProgramStates {
    NormalMode = 1,
    SetHours = 2,
    SetMinutes = 3,
    SetSeconds = 4,
}
fn f(foo: &MyProgramStates) -> u8 {
    *foo as u8
}

pub struct EPD<'a, SpiBus: 'static> {
    epd_spi: &'a CommonBus<SpiBus>,
    epd_timer: Timer<TIMER0>,
    epd: Epd2in7bc<&'a CommonBus<SpiBus>, P0_24<Output<PushPull>>, P0_17<Input<Floating>>, P0_23<Output<PushPull>>, P0_18<Output<PushPull>>, Timer<TIMER0>>,
    eink: Display2in7bc,
    text_style: TextStyle,
    character_style: SevenSegmentStyle<TriColor>,

}
pub struct SpiDevs<'a, SpiBus: 'static> {
    ds: DS1302<Reverser<&'a CommonBus<SpiBus>>, P0_04<Output<PushPull>>, MyClock<1000>, 1000>,
    //epd: Epd2in7bc<&'a CommonBus<SpiBus>, Pin<Output<PushPull>>, Pin<Input<Floating>>, Pin<Output<PushPull>>, P0_18<Output<PushPull>>, Timer<TIMER0>,
    epd: EPD<'a, SpiBus>,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ParkingTime {
    hour: u8,
    minute: u8,
}
trait ParkTime {
    fn get_parktime(&self) -> ParkingTime;
}

impl ParkTime for Clock {
    fn get_parktime(&self) -> ParkingTime {
        
        let h_park = (self.hours.hour().0 + if self.minutes > 29 {1} else {0}) % 12;
        let m_park = if self.minutes > 29 {0} else {30};
        
        ParkingTime{ hour: h_park, minute: m_park}
    }
}

#[app(device = crate::hal::pac, dispatchers = [SWI0])]
mod app {
    use crate::*;


    #[monotonic(binds = RTC1, default = true)]
    type MyMono = MonoRtc<RTC1>;
    
    #[shared]
    struct Shared {
        gpiote: Gpiote,
        appstate: MyProgramStates,
        rng: Rng,
        clock: Clock,
    
        spidevs: SpiDevs<'static, SpiBus>,
    }

    #[local]
    struct Local {
        rtc: Rtc<hal::pac::RTC0>,
        btn1: Pin<Input<PullUp>>,
        btn2: Pin<Input<PullUp>>,
        oled: Ssd1306<I2CInterface<Twi<TWI0>>, DisplaySize128x32, TerminalMode>,
        last_park: ParkingTime,
    }
    
    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("Initializing RTIC");
        let p = cx.device;
        let mut cp = cx.core;
        let mono = MonoRtc::new(p.RTC1);
        
        let port0 = p0::Parts::new(p.GPIO);
        let gpiote = Gpiote::new(p.GPIOTE);

        let clocks = hal::clocks::Clocks::new(p.CLOCK);
        let _clocks = clocks.start_lfclk();
        
        let rng = Rng::new(p.RNG);

        let btn1 = port0.p0_12.into_pullup_input().degrade();
        let btn2 = port0.p0_13.into_pullup_input().degrade();

        // swapped from Groove connector
        let oled_scl = port0.p0_05.into_floating_input().degrade();
        let oled_sda = port0.p0_06.into_floating_input().degrade();

        let spi_sck = port0.p0_29.into_push_pull_output(Level::Low).degrade();
        let spi_miso = port0.p0_28.into_floating_input().degrade();
        let spi_mosi = port0.p0_25.into_push_pull_output(Level::Low).degrade();
    
        let eink_busy = port0.p0_17.into_floating_input();
        let eink_rst = port0.p0_18.into_push_pull_output(Level::Low);
        let eink_dc = port0.p0_23.into_push_pull_output(Level::Low);
    
        let eink_cs = port0.p0_24.into_push_pull_output(Level::Low);
        let ds_cs = port0.p0_04.into_push_pull_output(Level::Low);

        gpiote
            .channel0()
            .input_pin(&btn1)
            .hi_to_lo()
            .enable_interrupt();
        gpiote
            .channel1()
            .input_pin(&btn2)
            .hi_to_lo()
            .enable_interrupt();
        
        let mut rtc = Rtc::new(p.RTC0, 60).unwrap();
        rtc.set_compare(RtcCompareReg::Compare0, hal::clocks::LFCLK_FREQ).unwrap();
        rtc.enable_event(RtcInterrupt::Compare0);
        rtc.enable_interrupt(RtcInterrupt::Compare0, Some(&mut cp.NVIC));

    
        let oled_i2c = twi::Twi::new(p.TWI0, twi::Pins {scl: oled_scl, sda: oled_sda}, twi::Frequency::K250);
        let interface = ssd1306::I2CDisplayInterface::new(oled_i2c);
        let mut oled = Ssd1306::new(interface, ssd1306::prelude::DisplaySize128x32, ssd1306::prelude::DisplayRotation::Rotate180)
            .into_terminal_mode();
        
        oled.init().ok();
        oled.clear().ok();
        let _ = oled.write_str("Rust").ok();
        rprintln!("OLED is cooked");
        
        
        let spi_p = spi::Spi::new(
            p.SPI1, 
            spi::Pins {
                sck: spi_sck,
                mosi: Some(spi_mosi), 
                miso: Some(spi_miso),
            }, 
            spi::Frequency::M1, 
            spi::MODE_0
        );
        let spi_bus_manager = shared_bus_rtic::new!(spi_p, SpiBus);
        
        let ds_timer: MyClock<1000> = MyClock::new();
        let ds_spi = spi_bus_manager.acquire();
        
        let reversed_ds_spi = reverser::new(ds_spi);
        
        let mut ds1302 = DS1302::new(reversed_ds_spi, ds_cs, ds1302_mode::Hour24, ds_timer).unwrap();
        ds1302.set_clock_mode(ds1302_mode::Hour24).unwrap();
        
        let mut epd_spi = spi_bus_manager.acquire();
        let mut epd_timer = Timer::new(p.TIMER0);
        let epd = Epd2in7bc::new( &mut epd_spi, eink_cs, eink_busy, eink_dc, eink_rst, & mut epd_timer).unwrap();
        
        let character_style = eg_seven_segment::SevenSegmentStyleBuilder::new()
            .digit_size(embedded_graphics::prelude::Size::new(50, 90))
            .segment_color(TriColor::Black)
            .segment_width(7)
            .build();
        let text_style = embedded_graphics::text::TextStyleBuilder::new()
            .alignment(embedded_graphics::text::Alignment::Left)
            .baseline(embedded_graphics::text::Baseline::Top)
            .build();
        let mut eink = Display2in7bc::default();
        eink.set_rotation(DisplayRotation::Rotate270);
        
        let epds = EPD{
            epd_spi, 
            epd_timer, 
            epd,
            eink,
            character_style, 
            text_style,
        };
        let spidevs:SpiDevs<SpiBus> = SpiDevs { ds: ds1302,  epd: epds};
        
        let appstate = MyProgramStates::NormalMode;
        let clock = Clock{hours: Hours::Hour24(0), minutes: 0, seconds: 0};

        update_screens::spawn_after(2000.millis()).ok();

        rprintln!("Enabling interrupts");
        gpiote.port().enable_interrupt();
        rprintln!("Starting RTC");
        rtc.enable_counter();
        (
            Shared {
                gpiote,
                rng,
                appstate,
                clock,
                spidevs: spidevs,
            }, 
            Local {
                rtc,
                btn1,
                btn2,
                oled,
                last_park: ParkingTime { hour: 0, minute: 0 },
            },
            init::Monotonics(mono)
        )
    }

    
    #[task(binds = GPIOTE, shared = [gpiote])]
    fn button_pressed(mut ctx: button_pressed::Context) {
        ctx.shared.gpiote.lock(|gpiote| {
            gpiote.reset_events();
            
            debounce::spawn_after(100.millis()).ok();
        });
    }

    #[task(shared = [gpiote, appstate, clock, spidevs], local = [btn1, btn2])]
    fn debounce(mut ctx: debounce::Context) {
        let btn1_pressed = ctx.local.btn1.is_low().unwrap();
        let btn2_pressed = ctx.local.btn2.is_low().unwrap();
        
        ctx.shared.appstate.lock(|appstate| {
            
            match (btn1_pressed, btn2_pressed, &appstate) {
                (true, true, MyProgramStates::NormalMode) => {
                    *appstate = MyProgramStates::SetHours;
                    // Read time from ds to clock
                    let clock = ctx.shared.clock;
                    let spidevs = ctx.shared.spidevs;
                    (clock, spidevs).lock(|clock, spidevs|{
                        let (cl, _cal) = spidevs.ds.get_clock_calendar().unwrap();
                        clock.hours = Hours::Hour24(cl.hours.hour().0);
                        clock.minutes = cl.minutes;
                        clock.seconds = cl.seconds;
                    });


                },
                (true, true, _) => {
                    *appstate = MyProgramStates::NormalMode;
                    rprintln!("Should write time to clock");
                    let clock = ctx.shared.clock;
                    let spidevs = ctx.shared.spidevs;
                    (clock, spidevs).lock(|clock, spidevs|{
                        spidevs.ds.set_clock(Clock { hours: Hours::Hour24(clock.hours.hour().0), minutes: clock.minutes, seconds: clock.seconds }).ok();
                    });
                },
                (true, false, x) => {
                    *appstate = match &x {
                        MyProgramStates::SetHours => MyProgramStates::SetMinutes,
                        MyProgramStates::SetMinutes => MyProgramStates::SetSeconds,
                        MyProgramStates::SetSeconds => MyProgramStates::SetHours,
                        _ => MyProgramStates::NormalMode,
                    };
                },
                (false, true, x) => {
                    ctx.shared.clock.lock(|clock| {
                    match &x {
                        MyProgramStates::SetHours => { clock.hours = Hours::Hour24((clock.hours.hour().0+1) % 24);},
                        MyProgramStates::SetMinutes => {clock.minutes = (clock.minutes + 1) % 60;},
                        MyProgramStates::SetSeconds => {clock.seconds = (clock.seconds + 1) % 60;},
                        _ => {},
                    };
                    });
                },
                
                (_, _, _) => {
                    rprintln!("Unknown");
                }
                
            }
           
        });
        if btn1_pressed || btn2_pressed {
            update_screens::spawn().ok();
        }
        
    }
    
    #[task(shared=[appstate, clock, spidevs, rng], local=[oled, last_state: u8 = 0, last_park])]
    fn update_screens(mut ctx: update_screens::Context) {
        let mut display_set_clock = false;
        ctx.shared.appstate.lock(|appstate| {
            let app_val = f(appstate);
            if *ctx.local.last_state != app_val {
                ctx.local.oled.clear().ok();
                *ctx.local.last_state = app_val;
            }    
            
            match appstate {    
                MyProgramStates::NormalMode => {
                    ctx.local.oled.set_position(0, 0).ok();
                    write!(ctx.local.oled, "Normal").ok();
                },
                MyProgramStates::SetHours => {
                    ctx.local.oled.set_position(0, 2).ok();
                    write!(ctx.local.oled, "__      ").ok();
                    display_set_clock = true;
                },
                MyProgramStates::SetMinutes => {
                    ctx.local.oled.set_position(0, 2).ok();
                    write!(ctx.local.oled, "   __   ").ok();
                    display_set_clock = true;
                },
                MyProgramStates::SetSeconds => {
                    ctx.local.oled.set_position(0, 3).ok();
                    write!(ctx.local.oled, "      __").ok();
                    display_set_clock = true;
                },
            }
        });
        if display_set_clock {
            ctx.shared.clock.lock(|clock| {
                ctx.local.oled.set_position(0, 1).ok();
                write!(ctx.local.oled, "{:02}:{:02}:{:02}", clock.hours.hour().0, clock.minutes, clock.seconds).ok();
            });
        }
        
        let spidevs = ctx.shared.spidevs;
        let rng = ctx.shared.rng;

        (spidevs, rng).lock(|spidevs, rng|{
            let (cl, _cal) = spidevs.ds.get_clock_calendar().unwrap();
            ctx.local.oled.set_position(0, 3).ok();
            write!(ctx.local.oled, "{:02}:{:02}:{:02}", cl.hours.hour().0, cl.minutes, cl.seconds).ok();

            let c_park = cl.get_parktime();
            if *ctx.local.last_park != c_park {
                rprintln!("Update parkingtime");
                ctx.local.oled.set_position(10, 3).ok();
                write!(ctx.local.oled, "{:02}:{:02}", c_park.hour, c_park.minute).ok();
                
                // update EINK
                let mut buf = [0u8; 8];
                let s: &str = format_no_std::show(
                    &mut buf,
                    format_args!("{}:{:02}", c_park.hour, c_park.minute),
                ).unwrap();
                
                // !TODO Make these use the actual display and font size
                let max_y = 164 - 91;
                let max_x = 262 - if  c_park.hour > 9 { 4*57 + 5 } else {3* 57 + 5};
                
                let x = rng.random_u8() as i32 % max_x;
                let y = rng.random_u8() as i32 % max_y;
                embedded_graphics::text::Text::with_text_style(
                    &s,
                    embedded_graphics::prelude::Point { x, y },
                    spidevs.epd.character_style,
                    spidevs.epd.text_style,
                ).draw(&mut spidevs.epd.eink).ok();
                
                spidevs.epd.epd.wake_up(&mut spidevs.epd.epd_spi, &mut spidevs.epd.epd_timer).unwrap();
                spidevs.epd.epd.update_and_display_frame(&mut spidevs.epd.epd_spi, spidevs.epd.eink.bw_buffer(), &mut spidevs.epd.epd_timer).unwrap();
                spidevs.epd.epd.sleep(&mut spidevs.epd.epd_spi, &mut spidevs.epd.epd_timer).unwrap();
                spidevs.epd.eink.clear(TriColor::White).ok();

                *ctx.local.last_park = c_park;
            } 
            
        });
    }

    #[task(binds = RTC0, local = [rtc], shared = [appstate])]
    fn every_minute(ctx: every_minute::Context) {
        update_screens::spawn().ok();

        ctx.local.rtc.reset_event(RtcInterrupt::Compare0);
        ctx.local.rtc.clear_counter();
    }

}

