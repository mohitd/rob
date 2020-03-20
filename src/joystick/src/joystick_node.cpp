//

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "joystick/joystick.h"

static constexpr float V_MAX = 1.0;
static constexpr float W_MAX = M_PI;

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle nh;

    joystick::JoystickInterface joystick;
    joystick::JoystickValues joystick_values{};
    geometry_msgs::TwistStamped velocity{};

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
    ros::Rate sample_rate(20);
    while (ros::ok()) {
        if (joystick.Sample(joystick_values)) {
            float normalized_linear
                = static_cast<float>(joystick_values.axis_1.y)/ joystick::JoystickValues::MAX_AXIS_VALUE;
            float normalized_angular
                = static_cast<float>(joystick_values.axis_1.x)/ joystick::JoystickValues::MAX_AXIS_VALUE;
            velocity.twist.linear.x = normalized_linear * V_MAX;
            velocity.twist.angular.z = normalized_angular * W_MAX;
            velocity.header.stamp = ros::Time::now();
            ROS_INFO_STREAM("JS: " << velocity);
        }
        velocity_pub.publish(velocity);
        ros::spinOnce();
        sample_rate.sleep();
    }

    return 0;
}

