#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "App/arm_control.h"
#include "App/arm_control.cpp"
#include "App/keyboard.h"
#include "App/play.h"
#include "App/solve.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <signal.h>
// #include "arm_control/PosCmd.h"
// #include "arm_control/JointControl.h"
// #include "arm_control/JointInformation.h"
// #include "arm_control/ChassisCtrl.h"
#include <geometry_msgs/PoseStamped.h>
int CONTROL_MODE=0; // 0 arx5 rc ，1 5a rc ，2 arx5 joint_control ，3 5a joint_control   4 arx5 pos_control  5 5a pos_control
command cmd;

volatile sig_atomic_t app_stopped = 0;
void sigint_handler(int sig);
void safe_stop(can CAN_Handlej);

void sigint_handler(int sig)
{
    (void)sig;
    app_stopped = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_node3"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    node.param<int>("control_mode", CONTROL_MODE, CONTROL_MODE);
    arx_arm ARX_ARM((int) CONTROL_MODE);

    std::string joint_topic;
    std::string end_topic;
    std::string command_topic;
    std::string manual_control_topic;
    std::string manual_control_status_topic;
    node.param<std::string>("joint_topic", joint_topic, "/cobot_magic/leader/joint_left");
    node.param<std::string>("end_topic", end_topic, "/cobot_magic/leader/end_left");
    node.param<std::string>("command_topic", command_topic, "/cobot_magic/leader/command_joint_left");
    node.param<std::string>(
        "manual_control_topic",
        manual_control_topic,
        "/cobot_magic/leader/manual_control_left");
    node.param<std::string>(
        "manual_control_status_topic",
        manual_control_status_topic,
        "/cobot_magic/leader/manual_control_status_left");
    ros::Publisher pub_manual_control_status =
        node.advertise<std_msgs::Bool>(manual_control_status_topic, 10);

    auto publish_manual_control_status = [&pub_manual_control_status](bool enabled)
    {
        std_msgs::Bool status_msg;
        status_msg.data = enabled;
        pub_manual_control_status.publish(status_msg);
    };

    bool manual_control_state_initialized = false;
    bool manual_control_enabled = false;
    auto set_manual_control = [
        &ARX_ARM,
        &manual_control_state_initialized,
        &manual_control_enabled,
        &publish_manual_control_status](bool enabled)
    {
        if (manual_control_state_initialized && manual_control_enabled == enabled)
        {
            publish_manual_control_status(enabled);
            return;
        }
        manual_control_state_initialized = true;
        manual_control_enabled = enabled;
        if (enabled)
        {
            ARX_ARM.manual_control_requested = true;
            ARX_ARM.control_mode = 0;
            ARX_ARM.init_kp = 10;
            ARX_ARM.init_kp_4 = 0;
            ARX_ARM.init_kd = 0;
            ARX_ARM.init_kd_4 = 0;
            ARX_ARM.init_kd_6 = 0;
            ARX_ARM.is_teach_mode = true;
            ARX_ARM.is_torque_control = true;
            ARX_ARM.teach2pos_returning = false;
            for (int i = 0; i < 6; i++)
            {
                ARX_ARM.prev_target_pos[i] = ARX_ARM.target_pos[i];
            }
            ROS_INFO("Cobot Magic leader left switched to manual-control mode.");
        }
        else
        {
            ARX_ARM.manual_control_requested = false;
            ARX_ARM.control_mode = 2;
            ARX_ARM.is_teach_mode = false;
            ARX_ARM.is_torque_control = false;
            ARX_ARM.teach2pos_returning = false;
            for (int i = 0; i < 7; i++)
            {
                ARX_ARM.ros_control_pos_t[i] = ARX_ARM.current_pos[i];
                ARX_ARM.target_pos[i] = ARX_ARM.current_pos[i];
            }
            ROS_INFO("Cobot Magic leader left switched to ROS joint-command mode.");
        }
        publish_manual_control_status(enabled);
    };

    ros::Subscriber sub_manual_control = node.subscribe<std_msgs::Bool>(
        manual_control_topic,
        10,
        [&set_manual_control](const std_msgs::Bool::ConstSharedPtr& msg)
        {
            set_manual_control(msg->data);
        });

    ros::Subscriber sub_joint = node.subscribe<sensor_msgs::JointState>(
        command_topic,
        10,
        [&ARX_ARM, &manual_control_state_initialized, &manual_control_enabled, &publish_manual_control_status](
            const sensor_msgs::JointState::ConstSharedPtr& msg)
        {
            if (msg->position.size() < 7)
            {
                ROS_WARN("Ignoring leader left JointState command with fewer than 7 positions.");
                return;
            }
            manual_control_state_initialized = true;
            manual_control_enabled = false;
            ARX_ARM.manual_control_requested = false;
            ARX_ARM.control_mode = 2;
            ARX_ARM.is_teach_mode = false;
            ARX_ARM.is_torque_control = false;
            ARX_ARM.teach2pos_returning = false;
            for (int i = 0; i < 7; i++)
            {
                ARX_ARM.ros_control_pos_t[i] = msg->position[i];
                if (i < static_cast<int>(msg->velocity.size()))
                {
                    ARX_ARM.ros_control_vel[i] = msg->velocity[i];
                }
                else
                {
                    ARX_ARM.ros_control_vel[i] = 0.0;
                }
            }
            publish_manual_control_status(false);
        });

    ros::Publisher pub_joint01 = node.advertise<sensor_msgs::JointState>(joint_topic, 10);
    // ros::Publisher pub_joint = node.advertise<arm_control::JointControl>("/joint_control2", 10);
    ros::Publisher pub_pose = node.advertise<geometry_msgs::PoseStamped>(end_topic, 10);


////////////////////////////////////////////
    arx5_keyboard ARX_KEYBOARD;

    ros::Rate loop_rate(200);
    can CAN_Handlej;
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    keyThread.detach();
    sleep(1);

    while(ros::ok() && !app_stopped)
    { 
        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);

        ARX_ARM.get_joint();
        if(!ARX_ARM.is_starting){
             cmd = ARX_ARM.get_cmd();
        }
        ARX_ARM.update_real(cmd);

///////topic///////////////////////////////////////

                        //发布关节信息
                        // arm_control::JointControl msg_joint;        

                        // for(int i=0;i<6;i++)
                        // {
                        //     msg_joint.joint_pos[i] = ARX_ARM.current_pos[i];
                        //     msg_joint.joint_vel[i] = ARX_ARM.current_vel[i];
                        //     msg_joint.joint_cur[i] = ARX_ARM.current_torque[i];
                        // }  

                        //     msg_joint.joint_vel[6] = ARX_ARM.current_vel[6];
                        //     msg_joint.joint_cur[6] = ARX_ARM.current_torque[6];
                        //     msg_joint.joint_pos[6]=ARX_ARM.current_pos[6]*12; // 映射放大
                        //     msg_joint.mode = ARX_ARM.arx5_cmd.key_t;
                        
                        // pub_joint.publish(msg_joint);

                        //发布末端位置
                        // arm_control::PosCmd msg_pos_back; 
                                   
                        // msg_pos_back.x      =ARX_ARM.solve.End_Effector_Pose[0];
                        // msg_pos_back.y      =ARX_ARM.solve.End_Effector_Pose[1];
                        // msg_pos_back.z      =ARX_ARM.solve.End_Effector_Pose[2];
                        // msg_pos_back.roll   =ARX_ARM.solve.End_Effector_Pose[3];
                        // msg_pos_back.pitch  =ARX_ARM.solve.End_Effector_Pose[4];
                        // msg_pos_back.yaw    =ARX_ARM.solve.End_Effector_Pose[5];
                        // msg_pos_back.gripper=ARX_ARM.current_pos[6];

                        // pub_pos.publish(msg_pos_back);

                        geometry_msgs::PoseStamped msg_pose;
                        msg_pose.header.stamp = ros::Time::now();
                        msg_pose.pose.position.x = ARX_ARM.solve.End_Effector_Pose[0];
                        msg_pose.pose.position.y = ARX_ARM.solve.End_Effector_Pose[1];
                        msg_pose.pose.position.z = ARX_ARM.solve.End_Effector_Pose[2];

                        msg_pose.pose.orientation.x = ARX_ARM.solve.End_Effector_Pose[3];
                        msg_pose.pose.orientation.y = ARX_ARM.solve.End_Effector_Pose[4];
                        msg_pose.pose.orientation.z = ARX_ARM.solve.End_Effector_Pose[5];
                        msg_pose.pose.orientation.w = ARX_ARM.current_pos[6];

                        pub_pose.publish(msg_pose);

                        sensor_msgs::JointState msg_joint01;
                        msg_joint01.header.stamp = msg_pose.header.stamp;
                        int8_t num_joint = 7;
                        msg_joint01.name.resize(num_joint);
                        msg_joint01.velocity.resize(num_joint);
                        msg_joint01.position.resize(num_joint);
                        msg_joint01.effort.resize(num_joint);
                        
                        for (int8_t i=0; i < 7; ++i)
                        {   
                            msg_joint01.name[i] = "joint" + std::to_string(i);
                            msg_joint01.velocity[i] = ARX_ARM.current_vel[i];
                            msg_joint01.effort[i] = ARX_ARM.current_torque[i];
                            
                            msg_joint01.position[i] = ARX_ARM.current_pos[i];
                            //if (i == 6) msg_joint01.position[i] *= 12;    // 夹爪传动比映射
                        }
                        pub_joint01.publish(msg_joint01);

///////////////////////////////////////////////////



        ros::spinOnce();
        loop_rate.sleep();
        
        CAN_Handlej.arx_1();
    }
    ROS_WARN("Cobot Magic leader left stopping motors.");
    CAN_Handlej.arx_2();
    usleep(100000);
    _exit(0);
    return 0;
}
