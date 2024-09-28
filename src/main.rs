#[allow(unused_imports)]
use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::sensor_msgs, pr_info,
};

use controllers::p9n_interface;
use motor_controller::udp_communication;
use omni_control::{Chassis, OmniSetting, Tire};
use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use std::{cell::RefCell, rc::Rc};
const CHASSIS: Chassis = Chassis {
    fl: Tire { id: 0, raito: 1. },
    fr: Tire { id: 1, raito: 1. },
    br: Tire { id: 2, raito: 1. },
    bl: Tire { id: 4, raito: 1. }, //なぜか３でするとモータが同じ方向にしか回らない
};

// const OMNI_DIA:f64 =  0.1;
const MAX_PAWER_INPUT: f64 = 160.;
const MAX_PAWER_OUTPUT: f64 = 1.;
const MAX_REVOLUTION: f64 = 5400.;

const OWN_PORT: &str = "50003";
const BROADCAST_ADDR: &str = "192.168.1.3:60000";

fn main() -> Result<(), DynError> {
    let mut omni_setting = OmniSetting {
        chassis: CHASSIS,
        max_pawer_input: MAX_PAWER_INPUT,
        max_pawer_output: MAX_PAWER_OUTPUT,
        max_revolution: MAX_REVOLUTION,
    };
    let required_max_pawer_output = Rc::new(RefCell::new(MAX_PAWER_OUTPUT));
    let d4s = Rc::new(RefCell::new(0));
    // for debug
    let _logger = Logger::new("robo1_2024_a");
    let ctx = Context::new()?;
    let mut selector = ctx.create_selector()?;
    let node = ctx.create_node("robo1_2024_a", None, Default::default())?;
    let subscriber_cmd = node.create_subscriber::<msg::Twist>("cmd_vel1", None)?;
    let subscriber_joy = node.create_subscriber::<sensor_msgs::msg::Joy>("rjoy1", None)?;

    let required_max_pawer_output_cmd = Rc::clone(&required_max_pawer_output);
    selector.add_subscriber(
        subscriber_cmd,
        Box::new(move |msg| {
            omni_setting.max_pawer_output = *required_max_pawer_output_cmd.borrow();
            let motor_power = omni_setting.move_chassis(msg.linear.x, msg.linear.y, msg.angular.z);
            for i in motor_power.keys() {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, *i, motor_power[i]);
            }
        }),
    );

    let required_max_pawer_output_joy = Rc::clone(&required_max_pawer_output);
    let d4s_joy = Rc::clone(&d4s);
    selector.add_subscriber(
        subscriber_joy,
        Box::new(move |msg| {
            let binding = sensor_msgs::msg::Joy::new().unwrap();
            let mut joy_c = p9n_interface::DualShock4Interface::new(&binding);
            joy_c.set_joy_msg(&msg);

            if joy_c.pressed_triangle() {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 5, 0.5);
            } else if joy_c.pressed_cross() {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 5, -0.5);
            } else {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 5, 0.);
            }

            if joy_c.pressed_r2() {
                *required_max_pawer_output_joy.borrow_mut() = MAX_PAWER_OUTPUT / 10.;
            } else {
                *required_max_pawer_output_joy.borrow_mut() = MAX_PAWER_OUTPUT;
            }

            if joy_c.pressed_circle() && *d4s.borrow() == 0 {
                udp_communication::send_pwm_udp(OWN_PORT, BROADCAST_ADDR, 6, -1.);
                *d4s.borrow_mut() = 1;
            }

            if !joy_c.pressed_circle() && *d4s.borrow() == 1 {
                *d4s.borrow_mut() = 0;
            }
        }),
    );

    loop {
        selector.wait()?;
    }
}
