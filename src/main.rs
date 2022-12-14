extern crate ev3dev_lang_rust;

mod two_link_arm;

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
        dist_threshold: 0.2
    };

    let path = [[1., 1.], [-1., 1.]];

    arm.follow_path(path.to_vec());

    Ok(())
}
