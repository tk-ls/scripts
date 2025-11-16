# Repository Guidelines

## Project Structure & Module Organization
- `rover.py` is the CLI orchestrating controller setup, logging, and runtime flags.
- Shared configuration and safety logic live in `config.py`, `error_handling.py`, and `rover_base.py`.
- Interfaces sit in `interfaces/` (`keyboard`, `gui`, `gesture`); models/config in `gesture_model.pkl` and `rover_config.json`.
- Tests reside in `test_rover_system.py`; Windows bootstrap scripts stay in `install.ps1`.

## Build, Test, and Development Commands
- Provision dependencies with `python -m pip install -r requirements.txt` (PowerShell on Windows works best).
- Simulate locally using `python rover.py --mode gui --target simulator`; swap to `--target physical --port 1234` for BLE hardware.
- Run config and diagnostics via `python rover.py --config-wizard` and `python rover.py --test-connection --target simulator`.
- Execute automated checks with `python -m pytest --cov=.`; for quick iteration run `python test_rover_system.py`.

## Coding Style & Naming Conventions
- Use 4-space indentation, type hints, and module docstrings; keep lines ≤100 chars.
- Adopt snake_case for functions/variables and PascalCase for classes; name controllers `*Controller`, interfaces `*Interface`.
- Format with `black .` and lint with `flake8`; resolve warnings rather than silencing them.

## Testing Guidelines
- Tests rely on `unittest` with `pytest` runners; extend `test_rover_system.py` or mirror its layout.
- Name new suites `test_<module>` and isolate hardware dependencies with mocks/stubs.
- Aim to cover config validation, safety fallbacks, and CLI argument parsing before merging.

## Commit & Pull Request Guidelines
- Recent commits use short lowercase subjects; move toward precise imperative lines under 72 chars (e.g., `refactor: tighten safety thresholds`).
- Group related work per commit (config, interface, tests) and document manual runs in the PR body.
- PRs should link issues, flag simulator vs physical impacts, and attach screenshots/logs when UI or gesture behavior changes.

## Security & Configuration Tips
- Keep BLE credentials, dataset captures, and personal configs out of version control.
- Stage changes via `python rover.py --config-wizard`, then commit `rover_config.json` only when defaults truly shift.
- Review safety toggles in `rover_base.py` before relaxing thresholds; explain mitigations in release notes.
