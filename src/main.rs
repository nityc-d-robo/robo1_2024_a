#[allow(unused_imports)]
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::sensor_msgs,
    pr_info,
};

use motor_controller::udp_communication;
use omni_control::{Chassis, OmniSetting, Tire};
use safe_drive::msg::common_interfaces::geometry_msgs::msg;

const CHASSIS: Chassis = Chassis {
    fl: Tire { id: 0, raito: 1. },
    fr: Tire { id: 1, raito: 1. },
    br: Tire { id: 2, raito: 1. },
    bl: Tire { id: 3, raito: 1. },
};

// const OMNI_DIA:f64 =  0.1;
const MAX_PAWER_INPUT: f64 = 160.;
const MAX_PAWER_OUTPUT: f64 = 999.;
const MAX_REVOLUTION: f64 = 5400.;

fn main() -> Result<(), DynError> {
    let omni_setting = OmniSetting {
        chassis: CHASSIS,
        max_pawer_input: MAX_PAWER_INPUT,
        max_pawer_output: MAX_PAWER_OUTPUT,
        max_revolution: MAX_REVOLUTION,
    };

    // for debug
    let _logger = Logger::new("robo1_2024_a");
    let ctx = Context::new()?;
    let mut selector = ctx.create_selector()?;
    let node = ctx.create_node("robo1_2024_a", None, Default::default())?;

    let subscriber_cmd = node.create_subscriber::<msg::Twist>("cmd_vel1", None)?;
    let subscriber_joy = node.create_subscriber::<sensor_msgs::msg::Joy>("rjoy1", None)?;


    selector.add_subscriber(
        subscriber_cmd,
        Box::new(move |msg| {
            let motor_power = omni_setting.move_chassis(msg.linear.x, msg.linear.y, msg.angular.z);

            for i in motor_power.keys() {
                udp_communication::send_pwm_udp("50003", "192.168.3:60000", *i, motor_power[i]);
            }
        }),
    );

    // selector.add_subscriber(
    //     subscriber_joy,
    //     Box::new(move |_msg| {
    //         todo!();
    //     }),
    // );

    loop {
        selector.wait()?;
    }
}
