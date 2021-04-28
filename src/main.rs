#![no_std]
#![no_main]

/// Right and left are looking from behind the robot towards the front.
use hal::pwm::{NoPins, PwmChannel, TIM1_CH2, TIM2_CH1, TIM2_CH2};
use mpu6050::{
    euler::Euler, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll,
};
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f3xx_hal as hal;
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::stm32;
//use stm32f3::stm32f302::{self, interrupt};
use stm32f3xx_hal::delay::Delay;
//use cortex_m::asm::delay;

use drogue_mpu_6050 as mpu6050;

struct DrivingController {
    rs: RightServo,
    ls: LeftServo
}

/*
impl DrivingController {
    fn new(gpioa: &mut hal::gpio::gpioa::Parts, , ch2: &PwmChannel<TIM2_CH2, NoPins>) {
        let mut ch1_with_pins = ch1.output_to_pa0(gpioa.pa0.into_af1(&mut gpioa.moder, &mut gpioa.afrl));
        let mut ch2_with_pins = ch2.output_to_pa1(gpioa.pa1.into_af1(&mut gpioa.moder, &mut gpioa.afrl));
        ch1_with_pins.enable();
        ch2_with_pins.enable();
        let mut ls = LeftServo {s: ch2_with_pins};
        let mut rs = RightServo {s: ch1_with_pins};
    }
}
*/

struct RightServo {
    s: stm32f3xx_hal::pwm::PwmChannel<stm32f3xx_hal::pwm::TIM2_CH1, stm32f3xx_hal::pwm::WithPins>,
    trim: i32,
}

struct LeftServo {
    s: stm32f3xx_hal::pwm::PwmChannel<stm32f3xx_hal::pwm::TIM2_CH2, stm32f3xx_hal::pwm::WithPins>,
    trim: i32,
}

impl RightServo {
    //fn new(p0: gpioa::P0, gpioa: &mut hal::gpio::gpioa::Parts, ch1: PwmChannel<TIM2_CH1, NoPins>) -> RightServo {
    //}
}

impl LeftServo {
    //fn new(gpioa: &mut hal::gpio::gpioa::Parts, ch2: &PwmChannel<TIM2_CH2, NoPins>) -> LeftServo {
    //}
}

// for trim. 50hz = 20ms.
// We're high for between 5 and 10% of that (min 1ms, max 2ms), so our effective range is 1000us (= 5%). (Input between 500 and 1000).
// 10 us = 5 trims at (20ms/10k = 2 us each).
// at a resolution of 10k, that's 1us per tick. So one trim is 10 ticks.
pub trait Servo {
    fn set_duty(&mut self, duty: i32);

    // takes a number between -100 and 100.
    fn set_duty_i8(&mut self, d: i8) {
        let input = d as i32;
        self.set_duty(((input + 100) * 500 / 200) + 500);
        // map a i8 to 500-1000.
    }
    //fn new<T>(gpioa: &mut hal::gpio::gpioa::Parts, channel: &PwmChannel<T, NoPins>) -> self;
}

impl Servo for RightServo {
    fn set_duty(&mut self, d: i32) {
        self.s.set_duty((d + self.trim) as u32);
    }
}

impl Servo for LeftServo {
    fn set_duty(&mut self, d: i32) {
        self.s.set_duty((d + self.trim) as u32); // trim moves it in increments of 1/100 of the total range on each tick.
    }
}

// wheelcontroller should take a % speed and convert it into the correct pulse for that servo.
struct WheelController {}

struct WheelCalibration {}

enum Direction {
    Positive = 1,
    Negative = -1,
}

const right_wc: WheelCalibration = WheelCalibration {};
const left_wc: WheelCalibration = WheelCalibration {};

impl DrivingController {}

#[entry]
fn main() -> ! {
    //hprintln!("Hello, world!").unwrap();

    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    //let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
    let mut flash = dp.FLASH.constrain();
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    let mut d = Delay::new(cp.SYST, clocks);
    d.delay_ms(1000 as u16);

    //hprintln!("Initialized gyroscope.").unwrap();

    //let sda = gpioa.pa9.into_af4(&mut gpioa.moder, &mut gpioa.afrh);
    //scl.into_pull_up_input(moder, pupdr)

    // sda.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    // let sda = gpioa.pa10.into_alternate_af4_open_drain();
    // let scl = gpioa.pa9.into_alternate_af4_open_drain();

    //let mut tim15 = hal::timer::Timer::tim15(dp.TIM15, 100.khz(), clocks, &mut rcc.apb2);
    //tim15.listen(Update);
    //tim15.wait();
    //tim15.clear_update_interrupt_flag();

    //let mut pa4 = gpioa.pa4.into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);

    //pa4.make_interrupt_source(&mut &mut syscfg);

    let (ch1, ch2, ch3, ch4) = stm32f3xx_hal::pwm::tim2(dp.TIM2, 10000, 50.hz(), &clocks); // 50hz, each pulse is 20ms. So a full is 10% and a none is 5%.
                                                                                           //ch1_with_pins.set_duty(60);
                                                                                           //let mut rs = RightServo::new(&mut gpioa, ch1);

    let mut ch1_with_pins =
    ch1.output_to_pa0(gpioa.pa0.into_af1(&mut gpioa.moder, &mut gpioa.afrl));
    ch1_with_pins.enable();
    let mut rs = RightServo {
        s: ch1_with_pins,
        trim: -8,
    };

    let mut ch2_with_pins =
    ch2.output_to_pa1(gpioa.pa1.into_af1(&mut gpioa.moder, &mut gpioa.afrl));
    ch2_with_pins.enable();
    let mut ls = LeftServo {
        s: ch2_with_pins,
        trim: -5,
    };

    let mut dc = DrivingController {ls, rs};

    //let mut ls = LeftServo::new(&mut gpioa, &ch2);
    d.delay_ms(1000 as u16);

    //calibrate(&mut ls, &mut d);
    //calibrate(&mut rs, &mut d);

    // drive these between 500 and 1000.

    // 13 or 15 is a lowest driving speed for these servos.
    autonomous(&mut d, &mut dc);
    loop {

    }

    /*
    loop {
        //ls.set_duty(750);
        rs.set_duty_i8(30);
        ls.set_duty_i8(-30);
        //rs.set_duty_i8(-10);
        d.delay_ms(2000 as u16);
        rs.set_duty_i8(-30);
        ls.set_duty_i8(30);
        d.delay_ms(2000 as u16);
        //ls.set_duty(-10); // 750 / 10000
        //d.delay_ms(1000 as u16);
    }
    */
    //hprintln!("Initialized PWM").unwrap();
}

struct DrivingSequence {}

struct Action {
    left: ServoCommand,
    right: ServoCommand,
    time: u32,
}

type ServoCommand = i8;


// forward is -30, 30

const auto: [Action; 5] = [Action {
    left: -60,
    right: 60,
    time: 2000,
}, Action { // turn clockwise 30 degrees
    left: -30,
    right: 0,
    time: 500,
}, Action { // drive backwards
    left: 60,
    right: -60,
    time: 1500,
}, Action { // turn counter clockwise 30 degrees
    left: 0,
    right: 30,
    time: 500,
}, Action {
    left: 30,
    right: -30,
    time: 1000,
}];

fn autonomous(d: &mut stm32f3xx_hal::delay::Delay, dc: &mut DrivingController) {
    for a in &auto {
        dc.ls.set_duty_i8(a.left);
        dc.rs.set_duty_i8(a.right);
        d.delay_ms(a.time);
    }

    dc.ls.set_duty_i8(0);
    dc.rs.set_duty_i8(0);
}

// 1 controller tick is 10us. at 50hz,

fn calibrate(s: &mut impl Servo, d: &mut stm32f3xx_hal::delay::Delay) {
    for i in 0..3 {
        // count it off
        s.set_duty(120);
        d.delay_ms(1000 as u16);
    }

    for i in 0..30 {
        s.set_duty(90 + i);
        d.delay_ms(1000 as u16);
    }
}
