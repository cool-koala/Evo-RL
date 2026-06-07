#!/usr/bin/env python
"""Run lerobot-train with episodes filtered by episode-level metadata."""

from __future__ import annotations

import argparse
import ast
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pandas as pd


def _pop_option(args: list[str], name: str) -> tuple[str | None, list[str]]:
    value = None
    kept: list[str] = []
    i = 0
    while i < len(args):
        arg = args[i]
        if arg == name:
            if i + 1 >= len(args):
                raise ValueError(f"Missing value for {name}")
            value = args[i + 1]
            i += 2
            continue
        if arg.startswith(f"{name}="):
            value = arg.split("=", 1)[1]
            i += 1
            continue
        kept.append(arg)
        i += 1
    return value, kept


def _get_option(args: list[str], name: str) -> str | None:
    value, _ = _pop_option(args, name)
    return value


def _parse_episodes(value: str | None) -> set[int] | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    if text.startswith("["):
        parsed = ast.literal_eval(text)
        return {int(item) for item in parsed}
    return {int(item.strip()) for item in text.split(",") if item.strip()}


def _resolve_dataset_root(train_args: list[str]) -> Path:
    root = _get_option(train_args, "--dataset.root")
    if root:
        return Path(root)

    repo_id = _get_option(train_args, "--dataset.repo_id")
    if not repo_id:
        raise ValueError("Pass --dataset.root or --dataset.repo_id in the lerobot-train arguments.")

    lerobot_home = os.environ.get("LEROBOT_HOME")
    base = Path(lerobot_home).expanduser() if lerobot_home else Path.home() / ".cache/huggingface/lerobot"
    return base / repo_id


def _load_episode_metadata(root: Path, metadata_key: str) -> pd.DataFrame:
    episodes_dir = root / "meta" / "episodes"
    paths = sorted(episodes_dir.glob("*/*.parquet"))
    if not paths:
        raise FileNotFoundError(f"No episode metadata parquet files found under {episodes_dir}")

    frames = []
    columns = ["episode_index", metadata_key]
    for path in paths:
        try:
            frames.append(pd.read_parquet(path, columns=columns))
        except Exception as exc:
            raise RuntimeError(
                f"Could not read episode metadata field '{metadata_key}' from {path}."
            ) from exc

    return pd.concat(frames, ignore_index=True)


def _select_episodes(
    root: Path,
    metadata_key: str,
    metadata_value: str,
    requested_episodes: set[int] | None,
) -> list[int]:
    episodes = _load_episode_metadata(root, metadata_key)
    expected = metadata_value.strip().lower()
    mask = episodes[metadata_key].astype(str).str.strip().str.lower().eq(expected)
    if requested_episodes is not None:
        mask &= episodes["episode_index"].astype(int).isin(requested_episodes)

    selected = sorted(episodes.loc[mask, "episode_index"].astype(int).tolist())
    if not selected:
        scope = "selected episodes" if requested_episodes is not None else "dataset"
        raise ValueError(f"No episodes in {scope} match {metadata_key}={metadata_value!r}.")
    return selected


def _format_episodes_arg(episodes: list[int]) -> str:
    return "[" + ",".join(str(ep) for ep in episodes) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Filter a LeRobot dataset by episode-level metadata, then forward the resulting "
            "--dataset.episodes list to lerobot-train."
        )
    )
    parser.add_argument("--metadata-key", default="episode_success")
    parser.add_argument("--metadata-value", default="success")
    parser.add_argument("--train-bin", default="lerobot-train")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("train_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()

    train_args = list(args.train_args)
    if train_args and train_args[0] == "--":
        train_args = train_args[1:]
    if not train_args:
        parser.error("Pass lerobot-train arguments after '--'.")

    explicit_episodes, train_args = _pop_option(train_args, "--dataset.episodes")
    requested_episodes = _parse_episodes(explicit_episodes)
    dataset_root = _resolve_dataset_root(train_args)
    selected_episodes = _select_episodes(
        dataset_root,
        metadata_key=args.metadata_key,
        metadata_value=args.metadata_value,
        requested_episodes=requested_episodes,
    )

    print(
        f"Selected {len(selected_episodes)} episodes from {dataset_root} "
        f"where {args.metadata_key}={args.metadata_value!r}: {selected_episodes}",
        flush=True,
    )

    train_bin = shutil.which(args.train_bin)
    if train_bin is None:
        raise FileNotFoundError(f"Could not find training executable: {args.train_bin}")

    command = [train_bin, *train_args, f"--dataset.episodes={_format_episodes_arg(selected_episodes)}"]
    if args.dry_run:
        print(" ".join(command))
        return 0
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
