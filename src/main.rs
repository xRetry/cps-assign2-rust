extern crate ev3dev_lang_rust;

mod two_link_arm;
mod kinematics;

use ev3dev_lang_rust::Ev3Result;
use ev3dev_lang_rust::motors::{MediumMotor, MotorPort};
use two_link_arm::TwoLinkArm;

fn main() -> Ev3Result<()> {

    let motors = [
        MediumMotor::get(MotorPort::OutA)?,
        MediumMotor::get(MotorPort::OutB)?
    ];

    let lengths = [14.3, 9.];

    let mut arm = TwoLinkArm{
        motors: motors,
        lengths: lengths,
        dist_threshold: 0.2,
        kin_forward: kinematics::kin_forward,
        kin_inverse: kinematics::kin_inv_analytic
    };

    let path = [[1., 1.], [-1., 1.]];

    arm.follow_path(path.to_vec());


    let motors = [
        MediumMotor::get(MotorPort::OutA)?,
        MediumMotor::get(MotorPort::OutB)?
    ];

    let mut arm2 = TwoLinkArm{
        motors: motors,
        lengths: lengths,
        dist_threshold: 0.2,
        kin_forward: kinematics::kin_forward,
        kin_inverse: kinematics::kin_inv_jac_trans
    };

    let path = [[1., 1.], [-1., 1.]];

    arm2.follow_path(path.to_vec());
    Ok(())
}
