#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import logging
import pickle  # nosec
import time
from concurrent import futures
from dataclasses import asdict
from pprint import pformat
from typing import Any, Protocol, runtime_checkable

import grpc
import torch

from lerobot.transport import (
    services_pb2,  # type: ignore
    services_pb2_grpc,  # type: ignore
)

from .configs import PolicyServerConfig
from .helpers import RemotePolicyConfig, TimedAction, TimedObservation
from .policy_server import PolicyServer


@runtime_checkable
class CloudPolicyAdapter(Protocol):
    """
    Minimal adapter interface for custom cloud-side algorithms.

    The adapter receives raw observations coming from the edge runtime and returns either:
    - a tensor/ndarray-like object shaped `(chunk, action_dim)` or `(action_dim,)`
    - a fully formed `list[TimedAction]` when the algorithm wants full timestamp control
    """

    def setup(self, policy_config: RemotePolicyConfig) -> None:
        """Called once after the edge client sends policy instructions."""

    def predict_action_chunk(
        self,
        observation: TimedObservation,
        policy_config: RemotePolicyConfig,
    ) -> torch.Tensor | list[TimedAction] | Any:
        """Predict an action chunk for the latest observation."""

    def reset(self) -> None:
        """Optional reset hook called whenever a new edge client session starts."""


class BaseCloudPolicyAdapter:
    """Small convenience base class for cloud policy adapters."""

    def setup(self, policy_config: RemotePolicyConfig) -> None:
        del policy_config

    def reset(self) -> None:
        return


class CustomPolicyServer(PolicyServer):
    """
    gRPC AsyncInference server for non-LeRobot cloud policies.

    This reuses the existing edge-to-cloud transport and observation queueing logic,
    but delegates action generation to a user-provided Python adapter.
    """

    prefix = "custom_policy_server"

    def __init__(self, config: PolicyServerConfig, adapter: CloudPolicyAdapter):
        super().__init__(config)
        self.adapter = adapter
        self.policy_specs: RemotePolicyConfig | None = None

    def _reset_server(self) -> None:
        super()._reset_server()
        try:
            self.adapter.reset()
        except Exception:
            self.logger.exception("Cloud policy adapter reset failed.")

    def SendPolicyInstructions(self, request, context):  # noqa: N802
        if not self.running:
            self.logger.warning("Server is not running. Ignoring policy instructions.")
            return services_pb2.Empty()

        client_id = context.peer()
        policy_specs = pickle.loads(request.data)  # nosec
        if not isinstance(policy_specs, RemotePolicyConfig):
            raise TypeError(f"Policy specs must be a RemotePolicyConfig. Got {type(policy_specs)}")

        self.policy_specs = policy_specs
        self.lerobot_features = policy_specs.lerobot_features
        self.actions_per_chunk = policy_specs.actions_per_chunk

        self.logger.info(
            "Receiving custom policy instructions from %s | "
            "Policy type: %s | Robot: %s | Action mode: %s | Action dim: %d | Device: %s",
            client_id,
            policy_specs.policy_type,
            policy_specs.robot_type,
            policy_specs.action_mode or "unspecified",
            len(policy_specs.action_feature_names),
            policy_specs.device,
        )
        self.adapter.setup(policy_specs)
        return services_pb2.Empty()

    def _normalize_adapter_result(
        self,
        result: torch.Tensor | list[TimedAction] | Any,
        observation_t: TimedObservation,
    ) -> list[TimedAction]:
        if isinstance(result, list):
            if not all(isinstance(item, TimedAction) for item in result):
                raise TypeError("When returning a list, every element must be a TimedAction.")
            return result

        action_tensor = torch.as_tensor(result)
        if action_tensor.ndim == 1:
            action_tensor = action_tensor.unsqueeze(0)
        elif action_tensor.ndim == 3:
            if action_tensor.shape[0] != 1:
                raise ValueError(
                    "Adapter action tensor with 3 dimensions must have batch size 1. "
                    f"Received shape {tuple(action_tensor.shape)}."
                )
            action_tensor = action_tensor.squeeze(0)
        elif action_tensor.ndim != 2:
            raise ValueError(
                "Adapter output must be 1D/2D action tensor or list[TimedAction]. "
                f"Received shape {tuple(action_tensor.shape)}."
            )

        if self.policy_specs is not None and self.policy_specs.action_feature_names:
            expected_action_dim = len(self.policy_specs.action_feature_names)
            if action_tensor.shape[-1] != expected_action_dim:
                raise ValueError(
                    f"Adapter action dimension mismatch: expected {expected_action_dim}, "
                    f"got {action_tensor.shape[-1]}."
                )

        action_tensor = action_tensor[: self.actions_per_chunk].detach().cpu()
        return self._time_action_chunk(
            observation_t.get_timestamp(),
            list(action_tensor),
            observation_t.get_timestep(),
        )

    def _predict_action_chunk(self, observation_t: TimedObservation) -> list[TimedAction]:
        if self.policy_specs is None:
            raise RuntimeError("No RemotePolicyConfig received yet. Wait for SendPolicyInstructions.")

        start_time = time.perf_counter()
        adapter_result = self.adapter.predict_action_chunk(observation_t, self.policy_specs)
        action_chunk = self._normalize_adapter_result(adapter_result, observation_t)
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        self.last_processed_obs = observation_t
        self.logger.info(
            "Observation %s | Adapter returned %d action(s) in %.2fms",
            observation_t.get_timestep(),
            len(action_chunk),
            elapsed_ms,
        )
        return action_chunk


def serve_with_adapter(adapter: CloudPolicyAdapter, cfg: PolicyServerConfig) -> grpc.Server:
    """
    Start a gRPC AsyncInference server using a custom adapter.

    Returns the started gRPC server instance so callers can decide how to block or stop it.
    """

    logging.info(pformat(asdict(cfg)))
    policy_server = CustomPolicyServer(cfg, adapter)
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    services_pb2_grpc.add_AsyncInferenceServicer_to_server(policy_server, server)
    server.add_insecure_port(f"{cfg.host}:{cfg.port}")
    policy_server.logger.info("CustomPolicyServer started on %s:%s", cfg.host, cfg.port)
    server.start()
    return server
