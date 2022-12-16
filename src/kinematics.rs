use crate::two_link_arm::TwoLinkArm;


pub fn kin_forward(arm: &mut TwoLinkArm, angles: [f64; 2]) -> [f64; 2] {
    return [
        arm.lengths[0] * angles[0].cos() + arm.lengths[1] * (angles[0]+angles[1]).cos(),
        arm.lengths[0] * angles[0].sin() + arm.lengths[1] * (angles[0]+angles[1]).sin()
    ];
}

pub fn kin_inv_analytic(arm: &mut TwoLinkArm, x_des: [f64; 2]) {
    let mut angles = [0.; 2];
    let mut sum_temp = (x_des[0].powi(2) + x_des[1].powi(2) - arm.lengths[0].powi(2) - arm.lengths[1].powi(2)) / (2.*arm.lengths[0]*arm.lengths[1]);

    if sum_temp > 1. { sum_temp = 1.; }
    if sum_temp < -1. { sum_temp = -1.; }

    angles[1] = -sum_temp.acos();
    angles[0] = (x_des[1] / x_des[0]).atan() - ((arm.lengths[1]*angles[1].sin()) / (arm.lengths[0]+arm.lengths[1]*angles[1].cos())).atan();

    if x_des[0] < 0. { angles[0] += std::f64::consts::PI; }

    arm.set_angles(angles);
}

pub fn kin_inv_jac_trans(arm: &mut TwoLinkArm, x_des: [f64; 2]) {
    let num_iter = 100;
    let step_size = 0.01;

    for _ in 0..num_iter {
        let angles_cur = arm.get_angles();
        let x_cur = (arm.kin_forward)(arm, angles_cur);

        let mut jac = jacobian(arm.lengths, angles_cur);

        let tmp = jac[0][1];
        jac[0][1] = jac[1][0];
        jac[1][0] = tmp;

        
        let mut x_diff = [0.; 2];
        for i in 0..2 {
            x_diff[i] = x_des[i] - x_cur[i];
        }

        let mut angles = [0.; 2];
        for i in 0..2 {
            let delta_angle = jac[i][0]*x_diff[0] + jac[i][1]*x_diff[1];
            angles[i] = step_size * delta_angle;
        }

        arm.set_angles(angles);
    }
}


fn jacobian(lengths: [f64; 2], angles: [f64; 2]) -> [[f64; 2]; 2] {
    return [
        [-lengths[0]*angles[0].sin() - lengths[1]*(angles[0]+angles[1]).sin(), -lengths[1]*(angles[0]+angles[1]).sin()],
        [lengths[0]*angles[0].cos() + lengths[1]*(angles[0]+angles[1]).cos(), lengths[1]*(angles[0]+angles[1]).cos()]
    ]
}
