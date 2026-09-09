#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidController {
    pub kp: f32, pub ki: f32, pub kd: f32, pub dt: f32,

    pub setpoint: f32,
    pub integral: f32,
    pub previous_error: f32,

    pub min: f32, pub max: f32,

    pub d_filter_alpha: f32,
    pub prev_derivative: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, dt: f32) -> Self {
        let dt: f32 = if dt.abs() < 1e-6 { 0.01 } else { dt };

        Self {
            kp, ki, kd, dt,

            setpoint: 0.0,
            integral: 0.0,
            previous_error: 0.0,

            min: -1e9, max: 1e9,

            d_filter_alpha: 1.0,
            prev_derivative: 0.0,
        }
    }

    pub fn update(&mut self, measurement: f32) -> f32 {
        if self.dt.abs() < 1e-6 {
            return 0.0;
        }

        let error: f32 = self.setpoint - measurement;

        let p: f32 = self.kp * error;

        let raw_d: f32 = (error - self.previous_error) / self.dt;
        let filtered_d: f32 = self.d_filter_alpha * raw_d + (1.0 - self.d_filter_alpha) * self.prev_derivative;
        self.prev_derivative = filtered_d;
        let d: f32 = self.kd * filtered_d;

        let without_i: f32 = p + d;

        let (i_min, i_max) = if self.ki.abs() > 1e-6 {
            let i_max = (self.max - without_i) / self.ki;
            let i_min = (self.min - without_i) / self.ki;
            if i_min < i_max {
                (i_min, i_max)
            } else {
                (i_max, i_min)
            }
        } else {
            (-1e6, 1e6)
        };

        self.integral += error * self.dt;
        self.integral = self.integral.clamp(-1e6, 1e6);
        self.integral = self.integral.clamp(i_min, i_max);

        let i: f32 = self.ki * self.integral;
        let output: f32 = without_i + i;

        self.previous_error = error;

        output.clamp(self.min, self.max)
    }

    pub fn set_derivative_filter(&mut self, alpha: f32) {
        self.d_filter_alpha = alpha.clamp(0.0, 1.0);
    }

    pub fn set_limits(&mut self, min: f32, max: f32) {
        self.min = min;
        self.max = max;
    }

    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_dt(&mut self, dt: f32) {
        if dt.abs() < 1e-6 {
            self.dt = 0.01;
        } else {
            self.dt = dt;
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.prev_derivative = 0.0;
    }
}