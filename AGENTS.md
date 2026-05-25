# Repository Guidelines

## Project Structure & Module Organization

Evo-RL builds on LeRobot and keeps Python source under `src/lerobot`. Core domains include `policies/`, `robots/`, `teleoperators/`, `datasets/`, `processor/`, `rl/`, `scripts/`, `motors/`, and `cameras/`. Dobot X-Trainer code lives in `src/lerobot/robots/dobot_xtrainer_follower` and `src/lerobot/teleoperators/dobot_xtrainer_leader`. Tests mirror these domains under `tests/`, with fixtures in `tests/fixtures` and hardware mocks in `tests/mocks`. Runnable workflows are in `examples/`; docs are in `docs/`; website assets are in `website/`; Docker files are in `docker/`.

## Build, Test, and Development Commands

- `conda create -y -n evo-rl python=3.10 && conda activate evo-rl`: create the Python environment.
- `pip install -e ".[dev,test]"`: install editable code with development and pytest extras.
- `git lfs pull`: fetch test artifacts and large assets.
- `pre-commit install`: enable local checks before commits.
- `pre-commit run --all-files`: run Ruff formatting/linting, typos, security, and mypy checks.
- `pytest -sv ./tests`: run the full test suite.
- `pytest -sv tests/<area>/test_<feature>.py`: run a focused test while developing.
- `make test-end-to-end DEVICE=cpu`: run policy train/eval smoke tests.
- `make build-user`: build the user Docker image.

## Coding Style & Naming Conventions

Use Python 3.10+. Ruff is the formatter and linter (`line-length = 110`, double quotes, spaces for indentation). Keep imports sorted by Ruff/isort with `lerobot` as first-party. Use `snake_case` for modules, functions, variables, and CLI options; `PascalCase` for classes and dataclasses; `config_<component>.py` for robot and teleoperator configs. Avoid generated files, caches, checkpoints, and local calibration data.

## Testing Guidelines

Tests use `pytest` with optional `pytest-cov` and `pytest-timeout`. Name tests `test_*.py` and colocate them with the relevant domain under `tests/`. Prefer mocks from `tests/mocks` for hardware behavior. Add or update tests for behavioral changes, new policies, robots, processors, and scripts. For hardware changes, include a unit test plus a manual command in the PR notes.

## Commit & Pull Request Guidelines

Recent history uses short imperative summaries, often Conventional Commits such as `feat(logging): add swanlab backend` and `fix(pi05): restore tied embed tokens`. Keep commits scoped and descriptive. PRs should follow `.github/PULL_REQUEST_TEMPLATE.md`: include motivation, behavior changes, linked issues, tests run, docs updates, and reviewer notes. Rebase on `main`, avoid working directly on `main`, and pass pre-commit plus relevant pytest commands before review.

## Security & Configuration Tips

Do not commit Hugging Face tokens, wandb credentials, robot serial paths, calibration files, datasets, or model outputs. Keep machine-specific paths in local shell history or ignored config, and document reproducible defaults in README or docs instead.
