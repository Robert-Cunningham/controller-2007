#![no_std]
#![no_main]

use mpu6050::{euler::Euler, quaternion::Quaternion, sensor::Mpu6050, yaw_pitch_roll::YawPitchRoll};
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
//use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use stm32f3xx_hal as hal;
use stm32f3xx_hal::stm32;
use stm32f3xx_hal::prelude::*;
use cortex_m_rt::entry;
//use cortex_m_semihosting::{hprintln};
//use stm32f3::stm32f302::{self, interrupt};
use stm32f3xx_hal::delay::Delay;
//use cortex_m::asm::delay;

use drogue_mpu_6050 as mpu6050;

struct DrivingController {
    left: stm32f3xx_hal::pwm::PwmChannel<stm32f3xx_hal::pwm::TIM2_CH2, stm32f3xx_hal::pwm::WithPins>,
    right: stm32f3xx_hal::pwm::PwmChannel<stm32f3xx_hal::pwm::TIM2_CH1, stm32f3xx_hal::pwm::WithPins>,
}

// wheelcontroller should take a % speed and convert it into the correct pulse for that servo.
struct WheelController {

}

struct WheelCalibration {

}

struct Servo<T> (
    stm32f3xx_hal::pwm::PwmChannel<T, stm32f3xx_hal::pwm::WithPins>
);

const right_wc: WheelCalibration = WheelCalibration {};
const left_wc: WheelCalibration = WheelCalibration {};

impl DrivingController {

}

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

    let scl = gpioa.pa10.into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr).into_af4(&mut gpioa.moder, &mut gpioa.afrh);
    //let scl = gpioa.pa10.into_af4_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let sda = gpioa.pa9.into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr).into_af4(&mut gpioa.moder, &mut gpioa.afrh);
    d.delay_ms(1000 as u16);
    let myi2c = hal::i2c::I2c::new(dp.I2C2, (sda, scl), 100.khz(), clocks, &mut rcc.apb1);
    d.delay_ms(1000 as u16);
    let mut sensor = mpu6050::sensor::Mpu6050::new(myi2c, mpu6050::address::Address::default()).unwrap();
    d.delay_ms(1000 as u16);
    init_dmp(&mut sensor, &mut d).unwrap();
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

    let (ch1, ch2, ch3, ch4) = stm32f3xx_hal::pwm::tim2(dp.TIM2, 1000, 50.hz(), &clocks); // 50hz, each pulse is 20ms. So a full is 10% and a none is 5%.
    let mut ch1_with_pins = ch1.output_to_pa0(gpioa.pa0.into_af1(&mut gpioa.moder, &mut gpioa.afrl));
    let mut ch2_with_pins = ch2.output_to_pa1(gpioa.pa1.into_af1(&mut gpioa.moder, &mut gpioa.afrl));
    ch1_with_pins.enable();
    ch2_with_pins.enable();
    ch1_with_pins.set_duty(60);
    d.delay_ms(1000 as u16);
    //hprintln!("Initialized PWM").unwrap();

    let mut yaw: f32 = 0.0;

    let mut counter = 0;

    loop {

        //let z = sensor.gyro().unwrap().z();
        let len = sensor.get_fifo_count().unwrap();
        let mut buf: [u8; 28] = [0; 28];
        if len >= 28 {
            counter = (counter + 1) % 10;
            let output = sensor.read_fifo(&mut buf[0..28]).unwrap();
            sensor.reset_fifo().unwrap();
            //yaw = euler.phi
            if counter == 0 {
                // hprintln!("{:.2}", z).unwrap();
                let q = Quaternion::from_bytes(&output[0..16]).unwrap().normalize();
                let ypw = YawPitchRoll::from(q);
                let euler = Euler::from(q);
                yaw = ypw.yaw;

                //hprintln!("ypw {:.2},{:.2},{:.2} euler {:.2},{:.2},{:.2}", ypw.yaw, ypw.pitch, ypw.roll, euler.phi, euler.psi, euler.theta).unwrap();
                //hprintln!("euler {:.2}, {:.2}, {:.2}", euler.phi, euler.psi, euler.theta).unwrap();
                if yaw > 0.0 {
                    //hprintln!("A").unwrap();
                    ch1_with_pins.set_duty(65);
                    ch2_with_pins.set_duty(85);
                } else {
                    //hprintln!("B").unwrap();
                    ch1_with_pins.set_duty(85);
                    ch2_with_pins.set_duty(65);
                }
            }
        }

        //ch1_with_pins.set_duty(70);
        //d.delay_ms(1000 as u16);
        //ch1_with_pins.set_duty(80);
        //d.delay_ms(1 as u16);
        //d.delay_us(100 as u16)
    }
}

//fn autonomous() {
//
//}

fn calibrate<T>(s: Servo<T>) {
    for i in 0..15 {
        //d.delay_ms(200 as u16);
        s.0.set_duty(85);
        //dc.left.
    }
}

use embedded_hal::blocking::i2c::{Write, WriteRead};

fn init_dmp<I2c>(sensor: &mut Mpu6050<I2c>, d: &mut stm32f3xx_hal::delay::Delay) -> Result<(), mpu6050::error::Error<I2c>>
    where I2c: Write + WriteRead,
          <I2c as WriteRead>::Error: core::fmt::Debug,
          <I2c as Write>::Error: core::fmt::Debug, {
        sensor.reset()?;
        d.delay_ms(200 as u16);
        sensor.disable_sleep()?;
        sensor.reset_signal_path()?;
        d.delay_ms(200 as u16);
        sensor.disable_dmp()?;
        sensor.set_clock_source(mpu6050::clock_source::ClockSource::Xgyro)?;
        d.delay_ms(200 as u16);
        sensor.disable_interrupts()?;
        sensor.set_fifo_enabled(mpu6050::fifo::Fifo::all_disabled())?;
        sensor.set_accel_full_scale(mpu6050::accel::AccelFullScale::G2)?;
        sensor.set_sample_rate_divider(4)?;
        sensor.set_digital_lowpass_filter(mpu6050::config::DigitalLowPassFilter::Filter1)?;
        sensor.load_firmware()?;
        sensor.boot_firmware()?;
        sensor.set_gyro_full_scale(mpu6050::gyro::GyroFullScale::Deg2000)?;
        sensor.enable_fifo()?;
        sensor.reset_fifo()?;
        sensor.disable_dmp()?;
        sensor.enable_dmp()?;

        Ok(())
}