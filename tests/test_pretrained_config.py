from __future__ import annotations

import json

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig


def test_from_pretrained_ignores_unknown_top_level_fields(tmp_path):
    config_dir = tmp_path / "checkpoint"
    config_dir.mkdir()

    reference_cfg = DiffusionConfig()
    reference_cfg._save_pretrained(config_dir)

    config_path = config_dir / "config.json"
    config = json.loads(config_path.read_text())
    config["resize_shape"] = [640, 480]
    config["crop_ratio"] = 0.8
    config["compile_model"] = True
    config["compile_mode"] = "reduce-overhead"
    config_path.write_text(json.dumps(config))

    loaded_cfg = PreTrainedConfig.from_pretrained(config_dir, local_files_only=True)

    assert isinstance(loaded_cfg, DiffusionConfig)
    assert loaded_cfg.n_obs_steps == reference_cfg.n_obs_steps
    assert loaded_cfg.horizon == reference_cfg.horizon
