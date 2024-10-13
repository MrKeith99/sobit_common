#include <queue>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <sobits_msgs/current_state.h>
#include "sobits_common/dynamixel/dynamixel_joint_control.h"
#include "sobits_common/dynamixel/dynamixel_port_control.h"
#include "sobits_common/dynamixel/dynamixel_setting.h"

namespace {
typedef struct {
    uint8_t  dxl_id;
    uint16_t dxl_current;
} CurrentRequest;

std::queue<CurrentRequest>                    current_request_queue;
dynamixel_port_control::DynamixelPortControl* driver_addr;

// [Real Robot]
void currentCtrlCallback(const sobits_msgs::current_state msg){
    std::string target_joint_name  = msg.joint_name;
    double      target_current_val = msg.current_ma;

    for( std::vector<dynamixel_control::DynamixelControl>::iterator it = driver_addr->joint_list_.begin(); it != driver_addr->joint_list_.end(); it++ ){
        if( it->getJointName() == target_joint_name ){
            CurrentRequest current_req;
            current_req.dxl_id      = it->getDxlId();
            current_req.dxl_current = it->current2DxlCurrent(target_current_val);
            current_request_queue.push(current_req);

            break;
        }
    }
}

}  // namespace

// [Isaac SIM] - Joint States Callback
std::map<std::string, double> joints_pos;
std::map<std::string, double> joints_vel;
std::map<std::string, double> joints_eff;
void cb_joints(const sensor_msgs::JointState joint_info){
    // Update the map of joint positions
    for (size_t i = 0; i < joint_info.name.size(); ++i) {
        joints_pos[joint_info.name[i]] = joint_info.position[i];
        joints_vel[joint_info.name[i]] = joint_info.velocity[i];
        joints_eff[joint_info.name[i]] = joint_info.effort[i];
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle                     nh;
    ros::NodeHandle                     pnh("~");
    // ros::Subscriber                     sub_current_ctrl = nh.subscribe("/current_ctrl", 10, &currentCtrlCallback); // [Real Robot]
    ros::Subscriber                     sub_joint_info = nh.subscribe("joint_states", 1, &cb_joints); // [Isaac SIM]
    dynamixel_setting::DynamixelSetting setting(nh);

    if( !setting.load() ) return -1;
    std::cout << "Dynamixel Setting Loaded" << std::endl;

    // [Isaac SIM] Wait for the joint states to be published
    do{
        ros::spinOnce();
    } while(!joints_pos.size() && !joints_vel.size() && !joints_eff.size());
    std::cout << "Joint States Loaded" << std::endl;

    dynamixel_port_control::DynamixelPortControl dynamixel_servo_control(nh, setting);
    controller_manager::ControllerManager        cm(&dynamixel_servo_control, nh);
    std::cout << "Controller Manager Initialized" << std::endl;

    // [Real Robot]
    // driver_addr = &dynamixel_servo_control;
    // dynamixel_servo_control.initializeSettingParam();

    // if( !dynamixel_servo_control.startUpPosition() ) return 0;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time     t  = dynamixel_servo_control.getTime();
    ros::Duration dt = dynamixel_servo_control.getDuration(t);

    std::cout << "Start Control Loop" << std::endl;

    while( ros::ok() ){
        dt = dynamixel_servo_control.getDuration(t);
        t  = dynamixel_servo_control.getTime();

        // Read the joint states
        // dynamixel_servo_control.read(t, dt); // [Real Robot]
        dynamixel_servo_control.read(t, dt); // [Isaac SIM]

        // Update the controller
        cm.update(t, dt);

        // Write the joint states
        // dynamixel_servo_control.write(t, dt); // [Real Robot]
        dynamixel_servo_control.write(dt); // [Isaac SIM]

        // [Real Robot] Set the current limits
        // while( current_request_queue.size() > 0 ){
        //     CurrentRequest req = current_request_queue.front();
        //     bool           res = dynamixel_servo_control.setCurrentLimit(req.dxl_id, req.dxl_current);

        //     if (res)  current_request_queue.pop();
        //     else break;
        // }
    }

    // Turn off the torque
    dynamixel_servo_control.setTorqueAll(false); // [Real Robot]

    // Stop the spinner
    spinner.stop();
}
