#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO("Calling drive robot\nlinear x: %1.2f angular z: %1.2f", lin_x, ang_z);
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    //Sets linear and angular motion for robot
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot!");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int mark1 = img.step/3;
    int mark2 = img.step*2/3;
    int white_pixel = 255;
    // Divides screen into three sections, uses position relative to sections to designate movement

    bool seen_ball = false;
    for (int i = 0; i < img.height*img.step; i++){
        if (img.data[i] == 255){
            int rowPos = i%img.step;
            ROS_INFO("Boundaries: %d %d Position: %d", mark1, mark2, rowPos);
            if (rowPos <= mark1){
                drive_robot(0.5, 0.5);
            } else if (rowPos <= mark2){
                drive_robot(1, 0);
            } else {
                drive_robot(0.5, -0.5);
            }
            seen_ball = true;
            ros::Duration(0.).sleep();
            break;
        }
    }

    if (seen_ball == false){
        drive_robot(0, 0);
    }
    // Reset if movement command wass issued
    seen_ball = false; 
    
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