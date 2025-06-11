# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to run an environment with sophisticated waving motion using absolute joint position control."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Waving agent for Isaac Lab environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch
import math

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg


def main():
    """Sophisticated waving agent with Isaac Lab environment."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")

    # reset environment
    obs = env.reset()

    # simulation parameters
    step_count = 0
    frequency = 1.0  # Hz
    amplitude = 0.5  # radians
    base_pose = torch.zeros(env.action_space.shape, device=env.unwrapped.device)

    # Get control timestep
    control_dt = getattr(env, "control_dt", 1.0 / 60.0)

    # Define joint indices per arm for waving
    # Assumed structure: [0-11] left arm, [12-23] right arm
    # Shoulder pitch, roll, elbow, wrist (example)
    left_arm_wave_joints = [1, 2, 4, 6]
    right_arm_wave_joints = [13, 14, 16, 18]

    # Define phase offsets per joint (to create a smooth wave)
    phase_offsets = [0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi]

    # simulate environment
    while simulation_app.is_running():
        with torch.inference_mode():
            t = step_count * control_dt

            # build wave pattern for each joint
            actions = base_pose.clone()
            for i, joint_idx in enumerate(left_arm_wave_joints):
                phase = phase_offsets[i]
                actions[..., joint_idx] = amplitude * math.sin(2 * math.pi * frequency * t + phase)
            for i, joint_idx in enumerate(right_arm_wave_joints):
                phase = phase_offsets[i]
                actions[..., joint_idx] = amplitude * math.sin(2 * math.pi * frequency * t + phase + math.pi)

            actions[..., 1] -= math.pi / 2
            actions[..., 13] -= math.pi / 2

            env.step(actions)
            step_count += 1


    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
