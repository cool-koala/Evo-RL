#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Starting Cobot Magic leader/follower ROS nodes."
echo "Leader topics:  /cobot_magic/leader/joint_left|right"
echo "Leader command: /cobot_magic/leader/command_joint_left|right"
echo "Leader manual:  /cobot_magic/leader/manual_control_left|right"
echo "Follower state: /cobot_magic/puppet/joint_left|right"
echo "Command topics: /cobot_magic/command/joint_left|right"

gnome-terminal -t "master2_left_leader" -- bash -c "cd '${workspace}/master2'; source devel/setup.bash && roslaunch arm_control arx5.launch control_mode:=0 command_topic:=/cobot_magic/leader/command_joint_left manual_control_topic:=/cobot_magic/leader/manual_control_left; exec bash;"
sleep 1
gnome-terminal -t "master1_right_leader" -- bash -c "cd '${workspace}/master1'; source devel/setup.bash && roslaunch arm_control arx5.launch control_mode:=0 command_topic:=/cobot_magic/leader/command_joint_right manual_control_topic:=/cobot_magic/leader/manual_control_right; exec bash;"
sleep 1

gnome-terminal -t "follow2_left_follower" -- bash -c "cd '${workspace}/follow2'; source devel/setup.bash && roslaunch arm_control arx5.launch control_mode:=2; exec bash;"
sleep 1
gnome-terminal -t "follow1_right_follower" -- bash -c "cd '${workspace}/follow1'; source devel/setup.bash && roslaunch arm_control arx5.launch control_mode:=2; exec bash;"

echo "Nodes started. Check with:"
echo "  rostopic hz /cobot_magic/leader/joint_left"
echo "  rostopic hz /cobot_magic/puppet/joint_left"
