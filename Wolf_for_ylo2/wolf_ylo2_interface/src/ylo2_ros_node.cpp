#include <ros/ros.h>
#include <thread>
#include <chrono>
#include "wolf_ylo2_interface/ylo2_ros_control.hpp"

static ylo22ros::ylo2ROSControl _ros_control;

int main(int argc, char**argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "ylo2_ros_control_node");

    // Load the task_period from the param server
    ros::NodeHandle nh;
    double period;
    if(!nh.getParam("/task_period",period))
    {
        ROS_ERROR("Task period not available in the ROS param server!");
        return 1;
    }

    // Starting the ros control
    _ros_control.init();

    // Start asynchronous ROS spinner
    ros::AsyncSpinner spinner(1); // Argument is the number of threads to use (0 means as many threads as processors)
    spinner.start();

    // Update duration of the loop
    ros::Duration loop_period(period);

    // Init
    auto start = std::chrono::system_clock::now();
    auto prev_start = start;
    auto end = start;
    double elapsed_seconds = 0.0;

    // Run the servo loop
    while (ros::ok())
    {

        start = std::chrono::system_clock::now();

        // For debugging
        //ROS_INFO_STREAM("Period: "<<std::chrono::duration_cast<
        //                std::chrono::duration<double> >(start - prev_start).count());

        // Updating the ros controller
        _ros_control.update(ros::Time::now(), loop_period);

        // Keep the ros magic alive
        ros::spinOnce();

        end = std::chrono::system_clock::now();

        elapsed_seconds = std::chrono::duration_cast<
          std::chrono::duration<double> >(end - start).count();

        // Sleep to keep the loop at specified period
        if(elapsed_seconds >= period)
            continue; // Do not sleep
        else
            std::this_thread::sleep_for( std::chrono::duration<double>(period - elapsed_seconds) ); // Sleep the remaining time

        prev_start = start;
    }

    spinner.stop();

    return 0;
}
