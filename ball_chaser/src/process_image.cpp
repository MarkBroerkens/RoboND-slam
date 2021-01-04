#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // sensor_msgs/Image.msg:
    // uint32 height         # image height, that is, number of rows
    // uint32 width          # image width, that is, number of columns
    // uint32 step           # Full row length in bytes
    // uint8[] data          # actual matrix data, size is (step * rows)

    int white_pixel = 255;
    int step_index = -1;

    // Loop through each pixel in the image and check if its white
    for (int i = 0; i < img.height * img.step; i=i+3) {
        if (img.data[i] >= white_pixel &&
            img.data[i+1] >= white_pixel &&
            img.data[i+2] >= white_pixel) {
            step_index = i % img.step;
            break;
            
        }
    }

    if (step_index < 0) { // ball not in frame
    	drive_robot(0.0, 0.1);
    }
    else
    {
      // Determining region: left, center, or right
      if (step_index < int(img.step/3))
      {
        drive_robot(0.0, 0.5);   // turn left bc white_pixel in left region
        ROS_INFO("Ball in LEFT Region. Turning left");
      }
      else if (step_index <= 2*int(img.step/3))
      {
      	drive_robot(0.5, 0.0);   // go forward
        ROS_INFO("Ball in CENTER. MOVING FORWARD");
      }
      else 
      {
        drive_robot(0.0, -0.5);   // turn right bc white_pixel in right region
        ROS_INFO("Ball in RIGHT Reguib, Turning right");
      }
        
    }
  

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
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
