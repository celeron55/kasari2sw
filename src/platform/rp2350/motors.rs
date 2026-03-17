use embassy_rp::pwm::{Config as PwmConfig, Pwm, PwmOutput, SetDutyCycle};

pub struct MotorController {
    left: Option<PwmOutput<'static>>,
    right: Option<PwmOutput<'static>>,
}

impl MotorController {
    pub fn new(
        pwm_slice: embassy_rp::peripherals::PWM_SLICE0,
        pin_left: embassy_rp::peripherals::PIN_0,
        pin_right: embassy_rp::peripherals::PIN_1,
    ) -> Self {
        let config = PwmConfig::default();

        let pwm = Pwm::new_output_ab(pwm_slice, pin_left, pin_right, config);
        let (left, right) = pwm.split();

        Self { left, right }
    }

    pub fn set_speeds(&mut self, left_rpm: f32, right_rpm: f32) {
        let left_duty = rpm_to_duty(left_rpm, 2500);
        let right_duty = rpm_to_duty(right_rpm, 2500);

        if let Some(ref mut left) = self.left {
            let _ = left.set_duty_cycle(left_duty);
        }
        if let Some(ref mut right) = self.right {
            let _ = right.set_duty_cycle(right_duty);
        }
    }
}

fn rpm_to_duty(rpm: f32, top: u16) -> u16 {
    let speed_percent = rpm / 2000.0 * 100.0;
    let duty_us = 1490.0 + 350.0 * speed_percent / 100.0;
    ((duty_us * 400.0 / 1_000_000.0) * top as f32) as u16
}
