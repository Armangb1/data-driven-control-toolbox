# AGENTS.md — Data-Driven Control Toolbox

## Quick Start
From MATLAB, with repo root as working directory:
```matlab
run('tools/devPath.m')   % adds toolbox/ and toolbox/lib to path
```
Do **not** add `toolbox/+ddc` itself to path; only `toolbox/` for package resolution.

## Key Commands
- **Run all tests:** `run('tests/runAllTests.m')`
- **Package toolbox:** `run('tools/buildToolbox.m')` → produces `Data-Driven-Control-Toolbox.mltbx`
- **Rebuild Simulink library:** `run('tools/buildLibraries.m')` → regenerates `toolbox/lib/ddc_lib.slx`

## Architecture
- All shipped code lives under `toolbox/+ddc/` (MATLAB package namespace).
- Categories: `+common`, `+deepc`, `+mfac`, `+spsa`, `+str`, `+ufc`, `+vrft`.
- Each category contains `matlab.System` classes (stateful, usable in Simulink or plain MATLAB) and helper functions.
- Simulink library: `toolbox/lib/ddc_lib.slx` — single browsable node in Library Browser, registered by `toolbox/lib/slblocks.m`.
- Library contents are defined in `tools/buildLibraries.m`; re-run after adding/removing blocks, or hand-edit in Simulink editor and re-save.

## Naming Conventions
- Package reference: `ddc.<category>.<Name>` (e.g., `ddc.deepc.DeePCController`).
- Simulink library files prefixed `ddc_*`.

## Testing
- Tests use `matlab.unittest` framework; each file in `tests/` is a test class.
- `runAllTests.m` discovers and runs all test files; adds toolbox path automatically.
- No external services required; tests are self-contained.

## Build Artifacts (gitignored)
- `*.mltbx`, `slprj/`, `*.slxc`, `*.autosave`, `*.asv`, `*.mex*`, `codegen/`, `release/`, `doc/html/`, `tests/results/`.

## Tips
- After modifying `matlab.System` classes, run tests to verify.
- After editing Simulink library blocks, either re-run `buildLibraries.m` or hand-save in Simulink editor.
- Packaging script uses Toolbox Packager API; output `.mltbx` is ready for distribution.
