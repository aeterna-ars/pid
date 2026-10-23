#[derive(Clone, Copy)]
pub struct PidController {
    pub setpoint: f32,
    kp: f32, 
    ki: f32, 
    kd: f32, 
    dt: f32,
    min: f32, 
    max: f32,
    d_filter_alpha: f32,
    
    integral: f32,
    previous_error: f32,
    prev_derivative: f32,
}

impl PidController {
    pub const fn new(kp: f32, ki: f32, kd: f32, dt: f32) -> Option<Self> {
        if dt <= 0.0 || !dt.is_finite() {
            return None;
        }
        
        Some(Self {
            kp, ki, kd, dt,
            setpoint: 0.0,
            d_filter_alpha: 1.0, // enabled by default
            min: f32::MIN,
            max: f32::MAX,
            integral: 0.0,
            previous_error: 0.0,
            prev_derivative: 0.0,
        })
    }

    pub fn set_limits(&mut self, min: f32, max: f32) -> Result<(), &'static str> {
        if min > max || min.is_nan() || max.is_nan() {
            return Err("NaN or incorrect");
        }
        self.min = min;
        self.max = max;
        Ok(())
    }

    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.d_filter_alpha = alpha.clamp(0.0, 1.0);
    }

    #[inline]
    pub fn update(&mut self, measurement: f32) -> f32 {
        if measurement.is_nan() {
            return f32::NAN; 
        }

        let error = self.setpoint - measurement;

        let p = self.kp * error;

        let raw_d = (error - self.previous_error) / self.dt;
        let filtered_d = self.d_filter_alpha * raw_d + (1.0 - self.d_filter_alpha) * self.prev_derivative;
        self.prev_derivative = filtered_d;
        let d = self.kd * filtered_d;

        let next_integral = self.integral + error * self.dt;
        let i = next_integral * self.ki;

        let output = p + i + d;

        let is_saturated = (output > self.max && error > 0.0) || (output < self.min && error < 0.0);
        
        if !is_saturated {
            self.integral = next_integral;
        }

        self.previous_error = error;

        output.clamp(self.min, self.max)
    }

    #[inline]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.prev_derivative = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-4;

    fn assert_near(a: f32, b: f32) {
        assert!((a - b).abs() < EPSILON, "Expected {}, got {}", b, a);
    }

    #[test]
    fn test_new_valid_and_invalid() {
        assert!(PidController::new(1.0, 1.0, 1.0, 0.01).is_some());
        assert!(PidController::new(1.0, 1.0, 1.0, 0.0).is_none());
        assert!(PidController::new(1.0, 1.0, 1.0, -0.01).is_none());
        assert!(PidController::new(1.0, 1.0, 1.0, f32::NAN).is_none());
    }

    #[test]
    fn test_set_limits() {
        let mut pid = PidController::new(1.0, 0.0, 0.0, 1.0).unwrap();
        assert!(pid.set_limits(-10.0, 10.0).is_ok());
        assert!(pid.set_limits(10.0, -10.0).is_err());
        assert!(pid.set_limits(f32::NAN, 10.0).is_err());
    }

    #[test]
    fn test_proportional_only() {
        let mut pid = PidController::new(2.0, 0.0, 0.0, 1.0).unwrap();
        pid.setpoint = 10.0;
        assert_near(pid.update(0.0), 20.0);
        assert_near(pid.update(5.0), 10.0);
    }

    #[test]
    fn test_integral_accumulation() {
        let mut pid = PidController::new(0.0, 0.5, 0.0, 2.0).unwrap();
        pid.setpoint = 10.0;
        assert_near(pid.update(0.0), 10.0); 
        assert_near(pid.update(0.0), 20.0); 
    }

    #[test]
    fn test_derivative_action() {
        let mut pid = PidController::new(0.0, 0.0, 1.0, 0.5).unwrap();
        pid.setpoint = 10.0;
        assert_near(pid.update(0.0), 20.0); 
        assert_near(pid.update(2.0), -4.0); 
    }

    #[test]
    fn test_derivative_with_filter() {
        let mut pid = PidController::new(0.0, 0.0, 1.0, 1.0).unwrap();
        pid.set_filter_alpha(0.5);
        pid.setpoint = 10.0;
        assert_near(pid.update(0.0), 5.0);
    }

    #[test]
    fn test_anti_windup_clamping() {
        let mut pid = PidController::new(0.0, 1.0, 0.0, 1.0).unwrap();
        pid.set_limits(-5.0, 5.0).unwrap();
        pid.setpoint = 10.0;
        
        assert_near(pid.update(0.0), 5.0);
        assert_near(pid.update(0.0), 5.0);
        
        pid.setpoint = 0.0;
        assert_near(pid.update(5.0), -5.0);
    }


    #[test]
    fn test_reset() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 1.0).unwrap();
        pid.setpoint = 10.0;
        pid.update(0.0);
        pid.reset();
        
        assert_near(pid.integral, 0.0);
        assert_near(pid.previous_error, 0.0);
        assert_near(pid.prev_derivative, 0.0);
    }

    #[test]
    fn test_nan_input_protection() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 1.0).unwrap();
        pid.setpoint = 10.0;
        assert!(pid.update(f32::NAN).is_nan());
    }
}