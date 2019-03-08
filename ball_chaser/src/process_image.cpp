#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//Define a global client that can request services

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("ROBOT is on the move ...");
    ball_chaser::DriveToTarget drive_cmd;
    drive_cmd.request.linear_x = lin_x;
    drive_cmd.request.angular_z = ang_z;
    
    if (!client.call(drive_cmd)){
        ROS_ERROR("Failed to execute drive command");
    }
}

//This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel_val = 255;
    int ahead = 0;
    int white_pixels = 0;
    int left = 0;
    int right = 0;
    int total_pixels = 0;
    float move_z;
    float x, z;
   for(int i=1; i<img.height*img.step; i++){ // Scan through the complete image frame
            ++total_pixels;
           if (img.data[i] == white_pixel_val){ // Bright WHITE Pixel found
                ++white_pixels;
                
              if (((i%img.step)<=(img.step/3)) && ((i%img.step) != 0)) { // WHITE Pixel on left of the image frame
                ++left; // Increment left pixel counter
              }
              else if (((i%img.step)<=(img.step/3)*2) && ((i%img.step) != 0)) { // WHITE Pixel on middle of the image frame
                ++ahead; // Increment middle pixel counter        
              }
              else { // WHITE Pixel on right of the image frame
                ++right; // Increment right pixel counter
              }
            }
   } 

// Finding x & z values for the drive command depending on the location of the white pixels

    if (white_pixels == 0) { //If NO WHITE Pixels found in the frame, rotate right with angular z = 0.5 and do not move (x = 0.0)
        x = 0.0;
        z = -0.5;
    }  
    else {
       if (left>ahead) { // More Pixels on the left than the middle frame
           if (ahead>right) { // More Pixels on the middle frame than the right frame
               move_z = -0.2; // Turn LEFT
           }
           else if (left>right) { // More Pixels on the left frame than right frame
               move_z = -0.2; // Turn LEFT
           }
           else {
               move_z = 0.2; // Turn RIGHT
           }
       }
       else {
           if (ahead > right) { // More Pixels on the middle frame than the right frame
               move_z = 0; // Do NOT turn
           }
           else { 
               move_z = 0.2; // Turn RIGHT
           }
       }
        x = 0.2;
        z = -move_z;
     }
        ROS_INFO("%d WHITE pixels found. Move velocity is %1.2f ", (int)white_pixels,z);
        ROS_INFO("Left = %d Ahead = %d Right = %d", left,ahead,right);
      
    // Command the ROBOT to move with requested velocity  
    drive_robot(x,z);

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
  
     // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  
    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

