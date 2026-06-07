# Repository Guidelines

## Project Structure & Module Organization
Core Python code lives under `src/lerobot/`, with policies, robots, teleoperators, cameras, datasets, RL utilities, processors, and CLI scripts grouped by domain. Examples are in `examples/`, documentation sources are in `docs/source/`, benchmark utilities are in `benchmarks/`, and the static project site is in `website/`. Docker build files are in `docker/`. Package assets, such as robot descriptions, are under `src/lerobot/assets/`. Third-party ROS runtime code is isolated in `third_party/cobot_magic_ros_runtime/`; avoid broad edits there unless the change is runtime-specific.

## Build, Test, and Development Commands
Install from source with `pip install -e .`; include development and test tools with `pip install -e ".[dev,test]"`. Run repository quality checks with `pre-commit run --all-files`. Run the Python test suite with `pytest -sv ./tests`. Use `make test-end-to-end DEVICE=cpu` for policy training and evaluation smoke tests. Build Docker images with `make build-user` or `make build-internal`.

## Coding Style & Naming Conventions
Target Python 3.10. Ruff handles formatting and linting with a 110-character line length, double quotes, space indentation, import sorting, pyupgrade, bugbear, naming, and simplification checks. Use `snake_case` for functions, variables, modules, and CLI options; use `PascalCase` for classes. Keep first-party imports grouped under `lerobot`, and avoid changing generated protobuf files (`*_pb2.py`, `*_pb2_grpc.py`) by hand.

## Testing Guidelines
Use `pytest` for Python tests. Add new tests under `tests/` with descriptive names such as `test_policy_config.py` or `test_robot_reset.py`. Prefer focused unit tests for config, processor, and utility changes. Add integration or Makefile smoke coverage for training, evaluation, dataset, or robot workflows. For hardware-only changes, document the device setup and manual validation steps in the PR.

## Commit & Pull Request Guidelines
Recent history uses short imperative commits and Conventional Commit-style scopes, for example `fix(cobot-magic): stabilize ROS HIL reset`. Prefer `type(scope): summary` when possible. PRs should include motivation, concrete changes, linked issues, testing commands or manual validation, documentation updates, and reviewer notes. Before review, run `pre-commit run -a` and the relevant `pytest` command.

## Security & Configuration Tips
Do not commit secrets, credentials, large generated artifacts, calibration caches, or local device-specific outputs. In examples, prefer stable device paths such as `/dev/serial/by-id/`, `/dev/v4l/by-id/`, or `/dev/v4l/by-path/` over transient numeric device names.
