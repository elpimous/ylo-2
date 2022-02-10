#include <iostream>
#include <string.h>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "k3lso_msgs/srv/motors_set_torque.hpp"

#include "moteus_pcan/moteus_pcan_controller.h"

using namespace std::chrono_literals;

struct MotorInfo{
    std::string joint_name;
    std::string can_interface;
    int can_id;
    double offset;
    bool invert;
};

std::vector<MotorInfo> motors_info = {
    // Front Right
    {"torso_to_abduct_fr_j",    "/dev/pcan-pcie_fd/devid=18", 1,  -0.00, true}, // Hip
    {"abduct_fr_to_thigh_fr_j", "/dev/pcan-pcie_fd/devid=12", 2,  0.92, true}, // Leg
    {"thigh_fr_to_knee_fr_j",   "/dev/pcan-pcie_fd/devid=11", 3,  -1.483529864, true}, // Low Leg
    // Front Left
    {"torso_to_abduct_fl_j",    "/dev/pcan-pcie_fd/devid=20", 4,  -0.00, true}, // Hip
    {"abduct_fl_to_thigh_fl_j", "/dev/pcan-pcie_fd/devid=10", 5,  0.92, false}, // Leg
    {"thigh_fl_to_knee_fl_j",   "/dev/pcan-pcie_fd/devid=13", 6,  -1.483529864, false}, // Low Leg
    // Rear Right
    {"torso_to_abduct_hr_j",    "/dev/pcan-pcie_fd/devid=19", 10, -0.00, false}, // Hip
    {"abduct_hr_to_thigh_hr_j", "/dev/pcan-pcie_fd/devid=15", 11, 1.047197551, true}, // Leg
    {"thigh_hr_to_knee_hr_j",   "/dev/pcan-pcie_fd/devid=17", 12, -1.483529864, true}, // Low Leg
    // Rear Left
    {"torso_to_abduct_hl_j",    "/dev/pcan-pcie_fd/devid=21", 7,  -0.00, false}, // Hip
    {"abduct_hl_to_thigh_hl_j", "/dev/pcan-pcie_fd/devid=14", 8,  1.047197551, false}, // Leg
    {"thigh_hl_to_knee_hl_j",   "/dev/pcan-pcie_fd/devid=16", 9,  -1.483529864, false}  // Low Leg
};

MoteusInterfaceMotorsMap interface_motors_map = {
    {"/dev/pcan-pcie_fd/devid=18", {1}}, // Hip FR
    {"/dev/pcan-pcie_fd/devid=12", {2}}, // Leg FR
    {"/dev/pcan-pcie_fd/devid=11", {3}}, // Low Leg FR
    {"/dev/pcan-pcie_fd/devid=20", {4}}, // Hip FL
    {"/dev/pcan-pcie_fd/devid=10", {5}}, // Leg FL
    {"/dev/pcan-pcie_fd/devid=13", {6}}, // Low Leg FL
    {"/dev/pcan-pcie_fd/devid=19", {10}}, // Hip RR
    {"/dev/pcan-pcie_fd/devid=15", {11}}, // Leg RR
    {"/dev/pcan-pcie_fd/devid=17", {12}}, // Low Leg RR
    {"/dev/pcan-pcie_fd/devid=21", {7}}, // Hip RL
    {"/dev/pcan-pcie_fd/devid=14", {8}}, // Leg RL
    {"/dev/pcan-pcie_fd/devid=16", {9}}  // Low Leg RL
};

MoteusPcanController controller(interface_motors_map);
 
bool running = true;

rclcpp::Node::SharedPtr node;
rclcpp::TimerBase::SharedPtr timer_joint_states;
rclcpp::TimerBase::SharedPtr timer_freqs;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states;
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_freqs;

void timer_joint_states_callback(){
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node->get_clock()->now();
    for(const auto& motor_info: motors_info){
        int motor_id = motor_info.can_id;
        float pos, vel, tor;
        controller._motors[motor_id]->get_feedback(pos, vel, tor);
        if(!motor_info.invert){
            msg.position.push_back((-6.28319*pos) - motor_info.offset);
        }else{
            msg.position.push_back((6.28319*pos) - motor_info.offset);
        }
        msg.velocity.push_back(vel);
        msg.effort.push_back(tor);
        msg.name.push_back(motor_info.joint_name);
    }
    publisher_joint_states->publish(msg);
}

void timer_freqs_callback(){
    std_msgs::msg::Int32MultiArray msg;
    msg.data = controller.get_freqs();
    publisher_freqs->publish(msg);
}

void set_torque_error(std::shared_ptr<k3lso_msgs::srv::MotorsSetTorque::Response> response, uint32_t error, 
                      const std::string& error_str){
    response->error = error;                          
    response->error_str = error_str;
    RCLCPP_ERROR(node->get_logger(), error_str);
}

void set_torque_callback(const std::shared_ptr<k3lso_msgs::srv::MotorsSetTorque::Request> request,
          std::shared_ptr<k3lso_msgs::srv::MotorsSetTorque::Response> response)
{
    bool ids_mode = false;
    if(request->ids.empty() && request->joint_names.empty()){
        set_torque_error(response, 1, "Neither ids nor joint_names were selected.");
        return;
    }
    if(!request->ids.empty() && !request->joint_names.empty()){
        set_torque_error(response, 2, "Only one ids or joint_names can be selected.");
        return;
    }
    if(request->ids.empty()){ // Joint Names mode
        RCLCPP_INFO(node->get_logger(), "Setting torques by Joints Names.");
        if(request->joint_names.size() != request->state.size()){
            set_torque_error(response, 3, "joints_names and states must be the same size.");
            return;
        }
    }else{ // IDs mode
        RCLCPP_INFO(node->get_logger(), "Setting torques by IDs.");
        if(request->ids.size() != request->state.size()){
            set_torque_error(response, 4, "ids and states must be the same size.");
            return;
        }
        ids_mode = true;
    }
    if(ids_mode){
        for(const auto& id: request->ids){
            // FIX!!!
            if(id<1 || id>12){
                set_torque_error(response, 5, "ID not valid." );
                return;
            }
        }
        for(size_t i=0; i<request->ids.size(); i++){
            auto id = request->ids[i];
            auto state = request->state[i];
            controller._motors[id]->set_torque_ena(state);
        }
    }else{
        RCLCPP_INFO(node->get_logger(), "Joint Names mode not implemented yet.");
    }
    response->error = 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("k3lso_moteus_node");

    if(!controller.is_initialized()){
        RCLCPP_FATAL(node->get_logger(), "Could not initialize Moteus controllers.");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moteus controllers intialized.");

    publisher_joint_states = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    publisher_freqs = node->create_publisher<std_msgs::msg::Int32MultiArray>("/moteus/freqs", 1);
    timer_joint_states = node->create_wall_timer(20ms, &timer_joint_states_callback);
    timer_freqs = node->create_wall_timer(1s, &timer_freqs_callback);
    
    rclcpp::Service<k3lso_msgs::srv::MotorsSetTorque>::SharedPtr torque_service = 
                        node->create_service<k3lso_msgs::srv::MotorsSetTorque>("/k3lso_moteus/set_torque", &set_torque_callback);

    RCLCPP_INFO(node->get_logger(), "Node running.");
    controller.start();
    
    while(rclcpp::ok()){
        if(!controller.all_running()){
            RCLCPP_FATAL(node->get_logger(), "Moteus controller stopped working.");
            break;
        }
        rclcpp::spin_some(node);
    }

    return 0; 
}
