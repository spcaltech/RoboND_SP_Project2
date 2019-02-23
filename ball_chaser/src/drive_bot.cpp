#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

// handle_drive_request: handles drive requests when drive_bot service is
// requested.
// Publishes linear x and angular velocities to the wheel joints
// Returns message with feedback on the wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
        ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest recieved- linear_x: %1.2f, angular_z: %1.2f",
            (float)req.linear_x, (float)req.angular_z);

    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    motor_command_publisher.publish(motor_command);
    res.msg_feedback = "Vel set to: linear_x:" + std::to_string((double)motor_command.linear.x) 
        + " angular_z: " + std::to_string((double)motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback); 
    return true;  
}

int main(int argc, char** argv)
{
    //initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    //Create a ROS nodehandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type
    // geometry_msgs::Twist on the robot actuation topic with a publishing
    // queue of size 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    
    ros::spin();

    return 0;
}


