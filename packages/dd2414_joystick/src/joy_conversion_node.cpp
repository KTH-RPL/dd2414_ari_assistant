#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <stdlib.h>

#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class JoyConversion
{
public:
    JoyConversion()
    {
        // Subscribe to the "joy" topic to get joystick inputs
        ros::NodeHandle private_nh("~");
        std::string joy_input;
        std::string conv_out;
        int timeout;
        private_nh.param<std::string>("joy_out",joy_input,"/joy");
        private_nh.param<std::string>("conversion_out",conv_out,"/cmd_vel");
        private_nh.param("timeout",timeout,5);

    
        joy_sub_ = nh_.subscribe(joy_input, 10, &JoyConversion::joyCallback, this);

        // Publisher for the robot's velocity command
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(conv_out, 10);

        // Initialize last time the joystick message was received
        last_time_ = ros::Time::now().toSec();
        joystick_timeout_ = ros::Duration(timeout);  // Timeout threshold: 1 second

        // Initialize the last received velocity to zero
        last_cmd_vel_.linear.x = 0.0;
        last_cmd_vel_.angular.z = 0.0;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // Update the time of the last received joystick message
        last_time_ = ros::Time::now().toSec();

        // Create a new Twist message for robot velocity control
        geometry_msgs::Twist cmd_vel;

        // Check for pure rotation (axes[0] to control rotation without linear motion)
        if (fabs(msg->axes[0]) > 0.1)  // Threshold to detect if the joystick is used for rotation
        {
            // Set linear velocity to 0 (no forward/backward movement)
            cmd_vel.linear.x = 0.0;
            // Set angular velocity based on axes[0], limited to 0.5
            cmd_vel.angular.z = std::max(std::min(msg->axes[0], 0.5f), -0.5f);
        }
        else
        {
            // Use axes[2] for linear velocity (forward/backward), limited to 1.0
            cmd_vel.linear.x = std::max(std::min(msg->axes[3], 1.0f), -1.0f);

            // Use axes[3] for angular velocity (left-right turning), limited to 1.0
            cmd_vel.angular.z = std::max(std::min(msg->axes[2], 1.0f), -1.0f);
        }

        // Store the last received valid velocity
        last_cmd_vel_ = cmd_vel;

        // Publish the velocity command
        cmd_vel_pub_.publish(cmd_vel);
    }

    void checkJoystickTimeout()
    {
        // Check if joystick input has timed out

        if (ros::Time::now().toSec() - last_time_ > joystick_timeout_.toSec() && joystick_timeout_.toSec() != 0.0)
        {
            // If no joystick input is received for more than the timeout period, stop the robot
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

            // Publish zero velocity to stop the robot
            cmd_vel_pub_.publish(cmd_vel);

            // Print a warning message
            ROS_WARN("Lost connection to the 'joy' topic. Stopping robot.");
        }
        else
        {
            // If joystick input is still valid, continue publishing the last velocity received
            cmd_vel_pub_.publish(last_cmd_vel_);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher cmd_vel_pub_;

    double last_time_;          // Last time joystick message was received
    ros::Duration joystick_timeout_; // Timeout duration for joystick input

    geometry_msgs::Twist last_cmd_vel_; // Last received velocity command
};

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "joy_conversion_node");

    // Create the JoyConversion object to handle the subscription and publishing
    JoyConversion joy_conversion_node;

    // Spin in a loop, checking joystick timeouts and continuously publishing velocity
    ros::Rate loop_rate(10);  // Loop at 10 Hz
    while (ros::ok())
    {
        // Check if the joystick input has timed out or lost connection
        joy_conversion_node.checkJoystickTimeout();

        // Process incoming messages and maintain the loop
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
