#!/usr/bin/env python
"""Copy a LeRobot video dataset and resize all video streams.

This keeps the parquet data and episode metadata unchanged, rewrites videos to a
new resolution, and updates meta/info.json image feature shapes.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path


def _video_keys(info: dict) -> list[str]:
    return [key for key, feature in info["features"].items() if feature.get("dtype") == "video"]


def _copy_without_videos(source_root: Path, output_root: Path, overwrite: bool) -> None:
    if output_root.exists():
        if not overwrite:
            raise FileExistsError(f"{output_root} already exists. Pass --overwrite to replace it.")
        shutil.rmtree(output_root)

    def ignore_videos(_dir: str, names: list[str]) -> set[str]:
        return {"videos"} if "videos" in names else set()

    shutil.copytree(source_root, output_root, symlinks=False, ignore=ignore_videos)


def _update_info(output_root: Path, height: int, width: int, codec: str) -> list[str]:
    info_path = output_root / "meta" / "info.json"
    with open(info_path) as f:
        info = json.load(f)

    video_keys = _video_keys(info)
    for key in video_keys:
        feature = info["features"][key]
        channels = int(feature["shape"][2])
        feature["shape"] = [height, width, channels]
        feature_info = feature.setdefault("info", {})
        feature_info["video.height"] = height
        feature_info["video.width"] = width
        feature_info["video.codec"] = codec
        feature_info["video.pix_fmt"] = "yuv420p"
        feature_info["video.channels"] = channels

    with open(info_path, "w") as f:
        json.dump(info, f, indent=4, ensure_ascii=False)
    return video_keys


def _resize_video(src: Path, dst: Path, height: int, width: int, crf: int, preset: str) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp_dst = dst.with_name(f"{dst.stem}.tmp{dst.suffix}")
    if tmp_dst.exists():
        tmp_dst.unlink()

    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(src),
        "-map",
        "0:v:0",
        "-an",
        "-vf",
        f"scale={width}:{height}:flags=bicubic,setsar=1",
        "-c:v",
        "libx264",
        "-preset",
        preset,
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(tmp_dst),
    ]
    subprocess.run(command, check=True)
    tmp_dst.replace(dst)


def resize_video_dataset(
    source_root: Path,
    output_root: Path,
    height: int,
    width: int,
    overwrite: bool,
    crf: int,
    preset: str,
) -> None:
    source_root = source_root.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    if source_root == output_root:
        raise ValueError("output-root must be different from source-root.")
    if not (source_root / "meta" / "info.json").exists():
        raise FileNotFoundError(f"Missing LeRobot metadata: {source_root / 'meta' / 'info.json'}")

    _copy_without_videos(source_root, output_root, overwrite=overwrite)
    video_keys = _update_info(output_root, height=height, width=width, codec="h264")

    resized = 0
    for video_key in video_keys:
        src_dir = source_root / "videos" / video_key
        if not src_dir.exists():
            raise FileNotFoundError(f"Missing video directory: {src_dir}")
        for src in sorted(src_dir.glob("**/*.mp4")):
            rel = src.relative_to(source_root)
            dst = output_root / rel
            print(f"Resizing {rel} -> {width}x{height}", flush=True)
            _resize_video(src.resolve(), dst, height=height, width=width, crf=crf, preset=preset)
            resized += 1

    if resized == 0:
        raise RuntimeError(f"No mp4 files were found under {source_root / 'videos'}.")
    print(f"Wrote resized dataset to {output_root}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--height", type=int, default=224)
    parser.add_argument("--width", type=int, default=224)
    parser.add_argument("--crf", type=int, default=23)
    parser.add_argument("--preset", default="veryfast")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    resize_video_dataset(
        source_root=args.source_root,
        output_root=args.output_root,
        height=args.height,
        width=args.width,
        overwrite=args.overwrite,
        crf=args.crf,
        preset=args.preset,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
