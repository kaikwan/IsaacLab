# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

import omni.log

from isaacsim.core.prims import XFormPrim
import isaaclab.utils.string as string_utils
from isaaclab.assets.asset_base import AssetBase
from isaaclab.managers.action_manager import ActionTerm

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from . import actions_cfg


class PackingAction(ActionTerm):

    cfg: actions_cfg.PackingActionCfg
    """The configuration of the action term."""

    _asset: XFormPrim
    """The asset on which the placement action is applied."""
    
    _clip: torch.Tensor
    """The clip applied to the input action."""

    def __init__(self, cfg: actions_cfg.PackingActionCfg, env: ManagerBasedEnv) -> None:
        # initialize the action term
        super().__init__(cfg, env)

        # get pose of the asset
        self._asset_state = self._asset.get_world_poses()
        

        # create tensors for raw and processed actions
        self._raw_actions = torch.zeros(self.num_envs, 8, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, 8, device=self.device)

        # parse clip
        if self.cfg.clip is not None:
            if isinstance(cfg.clip, dict):
                self._clip = torch.tensor([[-float("inf"), float("inf")]], device=self.device).repeat(
                    self.num_envs, self.action_dim, 1
                )
                index_list, _, value_list = string_utils.resolve_matching_names_values(self.cfg.clip, self._joint_names)
                self._clip[:, index_list] = torch.tensor(value_list, device=self.device)
            else:
                raise ValueError(f"Unsupported clip type: {type(cfg.clip)}. Supported types are dict.")

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return 8

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor):
        # store the raw actions
        self._raw_actions[:] = actions
        
        self._processed_actions = self._raw_actions.clone()

        # get the position and orientation
        # offset to center of the tote if zero actions 
        self._processed_actions[:, 1:4] += self._asset_state[0]
        # z offset to displace the object above the table
        self._processed_actions[:, 1:4] += torch.tensor([0, 0, 0.1], device=self.device)
        # # compute the command
        # if self.cfg.clip is not None:
        #     self._processed_actions = torch.clamp(
        #         self._processed_actions, min=self._clip[:, :, 0], max=self._clip[:, :, 1]
        #     )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = torch.zeros_like(self._raw_actions[env_ids])
    
    def apply_actions(self):
        # first index is object id, the rest are position and orientation for the object
        # get the object id
        object_ids = self._processed_actions[:, 0].long() + 1
        # get the position and orientation
        position = self._processed_actions[:, 1:4]
        # print("position", position)
        orientation = self._processed_actions[:, 4:8]
        # get the object
        for idx, object_id in enumerate(object_ids):
            asset = self._env.scene[f"object{object_id.item()}"]
            orientation[idx, 0] = 1
            asset.write_root_link_pose_to_sim(torch.cat([position[idx], orientation[idx]]), env_ids=torch.tensor([idx], device=self.device))
            asset.write_root_com_velocity_to_sim(torch.zeros(6, device=self.device), env_ids=torch.tensor([idx], device=self.device))