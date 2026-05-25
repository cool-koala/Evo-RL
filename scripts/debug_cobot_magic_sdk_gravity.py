#!/usr/bin/env python

from __future__ import annotations

import argparse
import time

import numpy as np

import arx5_interface as arx5


def _arr(values) -> np.ndarray:
    return np.asarray(values, dtype=np.float64)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Test ARX5 SDK built-in gravity compensation without explicit inverse-dynamics torque."
    )
    parser.add_argument("--interface", required=True, help="SocketCAN interface, e.g. can0.")
    parser.add_argument("--model", default="X5")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--mode", choices=["damping", "current_cmd"], default="current_cmd")
    parser.add_argument("--background-send-recv", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gravity-vector", nargs=3, type=float, default=None)
    parser.add_argument("--log-every-s", type=float, default=0.5)
    args = parser.parse_args()

    robot_config = arx5.RobotConfigFactory.get_instance().get_config(args.model)
    if args.gravity_vector is not None:
        robot_config.gravity_vector = _arr(args.gravity_vector)

    controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
        "joint_controller",
        robot_config.joint_dof,
    )
    controller_config.gravity_compensation = True
    controller_config.background_send_recv = args.background_send_recv
    controller_config.shutdown_to_passive = True

    controller = arx5.Arx5JointController(robot_config, controller_config, args.interface)
    controller.set_log_level(arx5.LogLevel.INFO)

    print(f"interface={args.interface} model={args.model} mode={args.mode}")
    print(f"motor_id={list(robot_config.motor_id)}")
    print(f"motor_type={[str(item) for item in robot_config.motor_type]}")
    print(f"gravity_vector={robot_config.gravity_vector}")
    print(f"gravity_compensation={controller_config.gravity_compensation}")
    print(f"background_send_recv={controller_config.background_send_recv}")
    print(f"default_kp={controller_config.default_kp}")
    print(f"default_kd={controller_config.default_kd}")

    state = controller.get_joint_state()
    print(f"initial_pos={state.pos()}")
    print(f"initial_vel={state.vel()}")
    print(f"initial_torque={state.torque()}")

    controller.set_to_damping()
    gain = controller.get_gain()
    print(f"gain_after_damping.kp={gain.kp()}")
    print(f"gain_after_damping.kd={gain.kd()}")

    start = time.monotonic()
    last_log = start
    loops = 0
    try:
        while time.monotonic() - start < args.duration_s:
            if not args.background_send_recv:
                controller.recv_once()
            state = controller.get_joint_state()
            if args.mode == "current_cmd":
                cmd = arx5.JointState(robot_config.joint_dof)
                cmd.pos()[:] = state.pos()
                cmd.vel()[:] = 0.0
                cmd.torque()[:] = 0.0
                cmd.gripper_pos = state.gripper_pos
                cmd.gripper_vel = 0.0
                cmd.gripper_torque = 0.0
                controller.set_joint_cmd(cmd)
                if not args.background_send_recv:
                    controller.send_recv_once()

            now = time.monotonic()
            loops += 1
            if now - last_log >= args.log_every_s:
                joint_cmd = controller.get_joint_cmd()
                print(
                    f"t={now - start:5.2f}s "
                    f"hz={loops / (now - last_log):5.1f} "
                    f"pos={np.round(state.pos(), 4)} "
                    f"vel={np.round(state.vel(), 4)} "
                    f"torque={np.round(state.torque(), 4)} "
                    f"cmd_torque={np.round(joint_cmd.torque(), 4)}"
                )
                loops = 0
                last_log = now
            time.sleep(0.002)
    finally:
        controller.set_to_damping()
        print("done: set_to_damping() called")


if __name__ == "__main__":
    main()
