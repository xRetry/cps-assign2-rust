use std::{thread, time};
use ev3dev_lang_rust::motors::MediumMotor;


pub struct TwoLinkArm {
    pub motors: [MediumMotor; 2],
    pub lengths: [f64; 2],
    pub dist_threshold: f64
}

impl TwoLinkArm {
    pub fn follow_path(&mut self, path: Vec<[f64; 2]>) {
        for x_des in path {
            self.to_coordinate(x_des);  
        } 
    }
    
    pub fn to_coordinate(&mut self, x_des: [f64; 2]) {
        let angles = kin_inverse(self.lengths, x_des);
        self.set_angles(angles);
        self.wait_till_target(x_des);
    }

    fn set_angles(&mut self, mut rad: [f64; 2]) {
        rad[1] = -rad[1];

        for i in 0..2 {
            let ticks_per_rot = self.motors[i].get_count_per_rot().unwrap();
            let ticks = rad[i].to_degrees() / 360. * ticks_per_rot as f64;
            self.motors[i].run_to_abs_pos(Some(ticks as i32)).unwrap();
        }
    }

    fn get_angles(&self) -> [f64; 2] {
        let mut angles = [0.; 2];

        for i in 0..2 {
            let ticks_per_rot = self.motors[i].get_count_per_rot().unwrap() as f64;
            let proportion = self.motors[i].get_position().unwrap() as f64 / ticks_per_rot;
            angles[i] = proportion * 360.;
        }

        angles[1] = -angles[1];
        return angles;
    }

    fn wait_till_target(&self, x_des: [f64; 2]) {
        let mut x_cur = kin_forward(self.lengths, self.get_angles());        

        while vec_norm(x_cur, x_des) > self.dist_threshold {
            thread::sleep(time::Duration::from_millis(10));
            x_cur = kin_forward(self.lengths, self.get_angles());
        }
    }

}

fn kin_forward(lengths: [f64; 2], angles: [f64; 2]) -> [f64; 2] {
    return [
        lengths[0] * angles[0].cos() + lengths[1] * (angles[0]+angles[1]).cos(),
        lengths[0] * angles[0].sin() + lengths[1] * (angles[0]+angles[1]).sin()
    ];
}

fn kin_inverse(lengths: [f64; 2], x_des: [f64; 2]) -> [f64; 2] {
    let mut angles = [0.; 2];
    let mut sum_temp = (x_des[0].powi(2) + x_des[1].powi(2) - lengths[0].powi(2) - lengths[1].powi(2)) / (2.*lengths[0]*lengths[1]);

    if sum_temp > 1. { sum_temp = 1.; }
    if sum_temp < -1. { sum_temp = -1.; }

    angles[1] = -sum_temp.acos();
    angles[0] = (x_des[1] / x_des[0]).atan() - ((lengths[1]*angles[1].sin()) / (lengths[0]+lengths[1]*angles[1].cos())).atan();

    if x_des[0] < 0. { angles[0] += std::f64::consts::PI; }

    return angles;
}

fn vec_norm(x1: [f64; 2], x2: [f64; 2]) -> f64 {
    return ((x1[0] - x2[0]).powi(2) + (x1[1] - x2[1]).powi(2)).sqrt();
}

