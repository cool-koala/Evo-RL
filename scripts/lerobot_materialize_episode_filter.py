#!/usr/bin/env python
"""Materialize a LeRobot dataset filtered by episode-level metadata.

The output dataset contains only matching episodes and has a recomputed
meta/stats.json aggregated from those same episodes.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

from lerobot.datasets.compute_stats import aggregate_stats
from lerobot.datasets.utils import write_stats


def _read_episode_metadata(root: Path) -> pd.DataFrame:
    paths = sorted((root / "meta" / "episodes").glob("*/*.parquet"))
    if not paths:
        raise FileNotFoundError(f"No episode metadata parquet files found under {root / 'meta' / 'episodes'}")
    return pd.concat((pd.read_parquet(path) for path in paths), ignore_index=True)


def _select_episode_indices(episodes: pd.DataFrame, key: str, value: str) -> list[int]:
    if key not in episodes.columns:
        raise KeyError(f"Episode metadata key {key!r} does not exist.")

    expected = value.strip().lower()
    mask = episodes[key].astype(str).str.strip().str.lower().eq(expected)
    selected = sorted(episodes.loc[mask, "episode_index"].astype(int).tolist())
    if not selected:
        raise ValueError(f"No episodes match {key}={value!r}.")
    return selected


def _write_parquet_like_source(df: pd.DataFrame, path: Path, source_schema: pa.Schema | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if source_schema is not None:
        schema = pa.schema([field for field in source_schema if field.name in df.columns])
        table = pa.Table.from_pandas(df, schema=schema, preserve_index=False)
    else:
        table = pa.Table.from_pandas(df, preserve_index=False)
    pq.write_table(table, path, compression="snappy", use_dictionary=True)


def _as_stats_value(value):
    if isinstance(value, np.ndarray) and value.dtype == object:
        try:
            return np.array(value.tolist())
        except ValueError:
            return value
    return value


def _reshape_image_stat(value: np.ndarray) -> np.ndarray:
    value = np.asarray(value)
    if value.shape == (3, 1, 1):
        return value
    if value.size == 3:
        return np.array(value.tolist(), dtype=np.float64).reshape(3, 1, 1)
    return value


def _episode_stats_from_row(row: pd.Series, features: dict) -> dict[str, dict[str, np.ndarray]]:
    stats: dict[str, dict[str, np.ndarray]] = {}
    for column, value in row.items():
        if not column.startswith("stats/"):
            continue
        _, feature_name, stat_name = column.split("/", 2)
        value = _as_stats_value(value)
        feature = features.get(feature_name, {})
        if feature.get("dtype") in ["image", "video"] and stat_name != "count":
            value = _reshape_image_stat(value)
        stats.setdefault(feature_name, {})[stat_name] = np.asarray(value)
    return stats


def _copy_or_link(src: Path, dst: Path, mode: str) -> None:
    if mode == "none":
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        return
    if mode == "copy":
        shutil.copy2(src, dst)
    elif mode == "symlink":
        dst.symlink_to(src.resolve())
    else:
        raise ValueError(f"Unsupported video mode: {mode}")


def _materialize_data(
    source_root: Path,
    output_root: Path,
    old_to_new_episode: dict[int, int],
) -> dict[int, dict[str, int]]:
    data_dir = source_root / "data"
    paths = sorted(data_dir.glob("*/*.parquet"))
    if not paths:
        raise FileNotFoundError(f"No data parquet files found under {data_dir}")

    episode_data_metadata: dict[int, dict[str, int]] = {}
    global_index = 0
    output_file_index = 0

    for src_path in paths:
        df = pd.read_parquet(src_path)
        mask = df["episode_index"].astype(int).isin(old_to_new_episode)
        if not mask.any():
            continue

        filtered = df.loc[mask].copy().reset_index(drop=True)
        filtered["episode_index"] = filtered["episode_index"].astype(int).map(old_to_new_episode)
        filtered["index"] = np.arange(global_index, global_index + len(filtered), dtype=np.int64)

        chunk_idx = output_file_index // 1000
        file_idx = output_file_index % 1000
        dst_path = output_root / "data" / f"chunk-{chunk_idx:03d}" / f"file-{file_idx:03d}.parquet"
        source_schema = pq.read_schema(src_path)
        _write_parquet_like_source(filtered, dst_path, source_schema)

        for new_episode_index in sorted(filtered["episode_index"].unique()):
            ep_df = filtered[filtered["episode_index"] == new_episode_index]
            episode_data_metadata[int(new_episode_index)] = {
                "data/chunk_index": chunk_idx,
                "data/file_index": file_idx,
                "dataset_from_index": int(ep_df["index"].min()),
                "dataset_to_index": int(ep_df["index"].max() + 1),
            }

        global_index += len(filtered)
        output_file_index += 1

    if not episode_data_metadata:
        raise RuntimeError("No frame data was written for the selected episodes.")
    return episode_data_metadata


def _copy_videos(
    source_root: Path,
    output_root: Path,
    info: dict,
    selected_episode_rows: pd.DataFrame,
    mode: str,
) -> None:
    if mode == "none":
        return

    video_path_template = info.get("video_path")
    if not video_path_template:
        return

    video_keys = [
        key for key, feature in info["features"].items() if feature.get("dtype") == "video"
    ]
    copied: set[Path] = set()

    for _, row in selected_episode_rows.iterrows():
        for video_key in video_keys:
            chunk_idx = int(row[f"videos/{video_key}/chunk_index"])
            file_idx = int(row[f"videos/{video_key}/file_index"])
            rel_path = Path(
                video_path_template.format(
                    video_key=video_key,
                    chunk_index=chunk_idx,
                    file_index=file_idx,
                )
            )
            if rel_path in copied:
                continue
            _copy_or_link(source_root / rel_path, output_root / rel_path, mode)
            copied.add(rel_path)


def _materialize_metadata(
    source_root: Path,
    output_root: Path,
    info: dict,
    all_episode_rows: pd.DataFrame,
    selected_episode_indices: list[int],
    old_to_new_episode: dict[int, int],
    episode_data_metadata: dict[int, dict[str, int]],
) -> tuple[int, int]:
    selected_rows = (
        all_episode_rows[all_episode_rows["episode_index"].astype(int).isin(selected_episode_indices)]
        .copy()
        .sort_values("episode_index")
        .reset_index(drop=True)
    )

    all_stats = []
    for row_index, row in selected_rows.iterrows():
        old_episode_index = int(row["episode_index"])
        new_episode_index = old_to_new_episode[old_episode_index]
        selected_rows.at[row_index, "episode_index"] = new_episode_index
        selected_rows.at[row_index, "meta/episodes/chunk_index"] = 0
        selected_rows.at[row_index, "meta/episodes/file_index"] = 0
        for key, value in episode_data_metadata[new_episode_index].items():
            selected_rows.at[row_index, key] = value
        all_stats.append(_episode_stats_from_row(row, info["features"]))

    episode_path = output_root / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    source_episode_path = sorted((source_root / "meta" / "episodes").glob("*/*.parquet"))[0]
    _write_parquet_like_source(selected_rows, episode_path, pq.read_schema(source_episode_path))

    stats = aggregate_stats(all_stats)
    feature_stats = {key: value for key, value in stats.items() if key in info["features"]}
    write_stats(feature_stats, output_root)

    total_episodes = len(selected_episode_indices)
    total_frames = int(selected_rows["length"].sum())
    return total_episodes, total_frames


def materialize_filtered_dataset(
    source_root: Path,
    output_root: Path,
    metadata_key: str,
    metadata_value: str,
    video_mode: str,
    overwrite: bool,
) -> tuple[list[int], int]:
    source_root = source_root.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    if source_root == output_root:
        raise ValueError("output-root must be different from source-root.")
    if output_root.exists():
        if not overwrite:
            raise FileExistsError(f"{output_root} already exists. Pass --overwrite to replace it.")
        shutil.rmtree(output_root)

    with open(source_root / "meta" / "info.json") as f:
        info = json.load(f)

    episode_rows = _read_episode_metadata(source_root)
    selected_episode_indices = _select_episode_indices(episode_rows, metadata_key, metadata_value)
    old_to_new_episode = {old_idx: new_idx for new_idx, old_idx in enumerate(selected_episode_indices)}

    output_root.mkdir(parents=True, exist_ok=False)
    (output_root / "meta").mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_root / "meta" / "tasks.parquet", output_root / "meta" / "tasks.parquet")

    episode_data_metadata = _materialize_data(source_root, output_root, old_to_new_episode)
    total_episodes, total_frames = _materialize_metadata(
        source_root,
        output_root,
        info,
        episode_rows,
        selected_episode_indices,
        old_to_new_episode,
        episode_data_metadata,
    )

    selected_rows = episode_rows[episode_rows["episode_index"].astype(int).isin(selected_episode_indices)]
    _copy_videos(source_root, output_root, info, selected_rows, video_mode)

    info["total_episodes"] = total_episodes
    info["total_frames"] = total_frames
    info["splits"] = {"train": f"0:{total_episodes}"}
    with open(output_root / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=4, ensure_ascii=False)

    return selected_episode_indices, total_frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--metadata-key", default="episode_success")
    parser.add_argument("--metadata-value", default="success")
    parser.add_argument("--video-mode", choices=["symlink", "copy", "none"], default="symlink")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    episode_rows = _read_episode_metadata(args.source_root)
    selected = _select_episode_indices(episode_rows, args.metadata_key, args.metadata_value)
    total_frames = int(
        episode_rows.loc[episode_rows["episode_index"].astype(int).isin(selected), "length"].sum()
    )
    print(
        f"Selected {len(selected)} episodes where {args.metadata_key}={args.metadata_value!r}: {selected}"
    )
    print(f"Selected frames: {total_frames}")

    if args.dry_run:
        return 0

    materialize_filtered_dataset(
        source_root=args.source_root,
        output_root=args.output_root,
        metadata_key=args.metadata_key,
        metadata_value=args.metadata_value,
        video_mode=args.video_mode,
        overwrite=args.overwrite,
    )
    print(f"Wrote filtered dataset to {args.output_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
