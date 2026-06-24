#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cleanup() {
    echo "Stopping Cobot Magic arm nodes; requesting motor power-off..."
    "${workspace}/tools/stop_arms.sh" || true
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "Refreshing CAN interfaces..."
"${workspace}/tools/can.sh"

FOLLOWER_INPUT="${FOLLOWER_INPUT:-leader}"
case "${FOLLOWER_INPUT}" in
    leader)
        left_follower_command_topic="/cobot_magic/leader/joint_left"
        right_follower_command_topic="/cobot_magic/leader/joint_right"
        ;;
    policy|command)
        left_follower_command_topic="/cobot_magic/command/joint_left"
        right_follower_command_topic="/cobot_magic/command/joint_right"
        ;;
    *)
        echo "Unsupported FOLLOWER_INPUT='${FOLLOWER_INPUT}'. Use 'leader' or 'policy'."
        exit 1
        ;;
esac

echo "Starting Cobot Magic leader/follower ROS nodes."
echo "Follower input mode: ${FOLLOWER_INPUT}"
echo "Leader topics:  /cobot_magic/leader/joint_left|right"
echo "Leader command: /cobot_magic/leader/command_joint_left|right"
echo "Leader manual:  /cobot_magic/leader/manual_control_left|right"
echo "Follower state: /cobot_magic/puppet/joint_left|right"
echo "Follower input: ${left_follower_command_topic}|${right_follower_command_topic}"

launch_terminal() {
    local title="$1"
    local command="$2"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal -t "${title}" -- bash -lc "${command}; exec bash"
    else
        echo "gnome-terminal not found; starting ${title} in background."
        bash -lc "${command}" &
    fi
}

launch_arm() {
    local title="$1"
    local arm_workspace="$2"
    shift 2
    local setup="set +u && source /opt/ros/jazzy/setup.bash && source '${workspace}/${arm_workspace}/install/setup.bash' && set -u"
    launch_terminal "${title}" "${setup} && ros2 launch arm_control arx5.launch.py $*"
}

launch_arm "master2_left_leader" master2 \
    control_mode:=0 \
    command_topic:=/cobot_magic/leader/command_joint_left \
    manual_control_topic:=/cobot_magic/leader/manual_control_left
sleep 1
launch_arm "master1_right_leader" master1 \
    control_mode:=0 \
    command_topic:=/cobot_magic/leader/command_joint_right \
    manual_control_topic:=/cobot_magic/leader/manual_control_right
sleep 1

launch_arm "follow2_left_follower" follow2 \
    control_mode:=2 \
    command_topic:="${left_follower_command_topic}"
sleep 1
launch_arm "follow1_right_follower" follow1 \
    control_mode:=2 \
    command_topic:="${right_follower_command_topic}"

echo "Nodes started. Check with:"
echo "  ros2 topic hz /cobot_magic/leader/joint_left"
echo "  ros2 topic hz /cobot_magic/puppet/joint_left"
if [[ "${FOLLOWER_INPUT}" == "leader" ]]; then
    echo "  ros2 topic info /cobot_magic/leader/joint_left  # should show leader publisher and follower subscriber"
else
    echo "  ros2 topic info /cobot_magic/command/joint_left  # should show follower subscriber"
fi
echo "Leaders need about 4-5 seconds to finish initialization before gravity compensation starts."
echo "Keep this terminal open; press Ctrl-C here to stop all arms and power off motors."

while true; do
    sleep 1
done
