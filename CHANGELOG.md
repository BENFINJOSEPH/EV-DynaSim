# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)
and the project uses [Semantic Versioning](https://semver.org/).

## [1.1.0]

### Added
- DC fast-charging model (`src/evdynasim/charging.py`): BMS charge strategy with
  a C-rate limit derated by battery temperature and SOC, a charger power limit and a
  constant-voltage limit; active cooling; CC / CC* / CP / CV phase tracking.
- `charging` section in `configs/generic_ev.yaml` (charger, BMS derating tables, cooling).
- CLI: `evdynasim charge` (single session) and `evdynasim study charging`
  (ambient-temperature and charger-power studies); `evdynasim all` now includes charging.
- Charging plots: session time series, charging curves, study summary.
- Requirements-based test specification for charging (`tests/test_charging.py`,
  13 requirements, 46 cases); test suite grows from 98 to 144 tests.
- Docs: `docs/00_project_story.md` and `docs/09_charging.md`.

### Changed
- `battery.step` takes an optional `extra_heat_removal_w` (active cooling while
  charging); the default of 0 keeps the driving model identical to the C++ engine.
- `.gitignore` also ignores Visual Studio files (`.vs/`, `CMakeSettings.json`, `CMakeUserPresets.json`).

## [1.0.0]

### Added
- First release: longitudinal vehicle model, powertrain with motor loss map, 1-RC battery
  model with thermal model, backward and forward (PI driver) simulation, calibration and
  validation pipeline, sensitivity and ambient studies, C++17 engine with C API, CLI and CI.

[1.1.0]: ../../compare/v1.0.0...v1.1.0
[1.0.0]: ../../releases/tag/v1.0.0
