fn initialize_gyro() {
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
}

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

fn gyro_loop() {
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
}