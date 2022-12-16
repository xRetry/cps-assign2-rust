use std::{thread, time};
use ev3dev_lang_rust::motors::MediumMotor;


pub struct TwoLinkArm {
    pub motors: [MediumMotor; 2],
    pub lengths: [f64; 2],
    pub dist_threshold: f64,
    pub kin_forward: fn(&mut TwoLinkArm, [f64; 2]) -> [f64; 2],
    pub kin_inverse: fn(&mut TwoLinkArm, [f64; 2])

}

impl TwoLinkArm {
    pub fn follow_path(&mut self, path: Vec<[f64; 2]>) {
        for x_des in path {
            self.to_coordinate(x_des);  
        } 
    }
    
    pub fn to_coordinate(&mut self, x_des: [f64; 2]) {
        (self.kin_inverse)(self, x_des);
    }

    pub fn set_angles(&mut self, mut rad: [f64; 2]) {
        let x_des = (self.kin_forward)(self, rad);
        rad[1] = -rad[1];

        for i in 0..2 {
            let ticks_per_rot = self.motors[i].get_count_per_rot().unwrap();
            let ticks = rad[i].to_degrees() / 360. * ticks_per_rot as f64;
            self.motors[i].run_to_abs_pos(Some(ticks as i32)).unwrap();
        }
        self.wait_till_target(x_des);
    }

    pub fn get_angles(&self) -> [f64; 2] {
        let mut angles = [0.; 2];

        for i in 0..2 {
            let ticks_per_rot = self.motors[i].get_count_per_rot().unwrap() as f64;
            let proportion = self.motors[i].get_position().unwrap() as f64 / ticks_per_rot;
            angles[i] = proportion * 360.;
        }

        angles[1] = -angles[1];
        return angles;
    }

    fn wait_till_target(&mut self, x_des: [f64; 2]) {
        let mut x_cur = (self.kin_forward)(self, self.get_angles());        

        while vec_norm(x_cur, x_des) > self.dist_threshold {
            thread::sleep(time::Duration::from_millis(10));
            x_cur = (self.kin_forward)(self, self.get_angles());
        }
    }

}


fn vec_norm(x1: [f64; 2], x2: [f64; 2]) -> f64 {
    return ((x1[0] - x2[0]).powi(2) + (x1[1] - x2[1]).powi(2)).sqrt();
}

