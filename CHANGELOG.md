# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Added
- `ddc.mfac.MFACController`: generalized commit-form MFAC to the Full-Form
  Dynamic Linearization (FFDL) scheme. CFDL and PFDL are now parameter presets:
  `CFDL <=> Ly=0, Lu=1`, `PFDL <=> Ly=0, Lu=L`, `FFDL <=>` general `Ly, Lu`.
  New `Ly`/`Lu` pseudo-orders drive a vector PPD/PG estimate (projection
  algorithm + reset) and the full Hou & Jin control law with both delta-y and
  delta-u terms; `PhiInit`/`Rho` accept scalars or vectors of length `Ly+Lu`,
  and the second output remains the scalar `phi(Ly+1)` for backward
  compatibility. `tests/tMFACController.m` now covers CFDL regression (golden),
  PFDL/FFDL equivalence against an independent reference, the `phi(Ly+1)` reset
  rule, the K.N.Toosi plant example, and input validation.
- `+ddc/+str/+mdpp`: Minimum-Degree Pole Placement design engine (discrete-time,
  Astrom-Wittenmark Algorithm 3.1) for self-tuning regulators. Polynomial-based
  controller design via B-factoring, Diophantine (Sylvester) solving, and causality
  checks, with structured `MDPP:*` error identifiers and a unit-test class
  (`tests/MDPPDesignTest.m`). Resolves as `ddc.str.mdpp.*` and is included
  in the `.mltbx` build automatically.
- `ddc.str.mdpp`: `mdpp_design` and the public `toPoly` helper accept SISO `tf`, `zpk`,
  and `ss` model objects (Control System Toolbox) for the `A`, `B`, `Am`, `Bm`, `Ao`
  inputs, with `B` derivable from a model object. Numeric polynomial inputs and full
  base-MATLAB operation are unchanged.

## [0.1.0]

### Added
- Initial project scaffolding: `+ddc` package namespace, Simulink library set, tests, and dev tools.
- `ddc.common`: Hankel matrix builder, persistency-of-excitation check, sliding-window data buffer,
  recursive least squares (RLS) estimator, PRBS/multisine excitation signal generator.
- `ddc.deepc`: Data-Enabled Predictive Control (`DeePCController`) + offline `deepcDesign` helper.
- `ddc.mfac`: Compact-Form Dynamic Linearization Model-Free Adaptive Control (`MFACController`).
- `ddc.str`: Direct and Indirect self-tuning regulators (baseline adaptive control).
- `ddc.ufc`: Unfalsified adaptive switching control, single- and multimodel candidate-bank variants.
- `ddc.spsa`: Simultaneous Perturbation Stochastic Approximation online optimizer.
- `ddc.vrft`: Virtual Reference Feedback Tuning (offline, MATLAB-only).
- Single consolidated Simulink library (`ddc_lib.slx`) with all blocks organized into category
  subsystem folders (Common Utilities, DeePC, MFAC, Unfalsified Switching, SPSA, STR Baselines),
  registered as one "Data-Driven Control Toolbox" node in the Library Browser via
  `toolbox/lib/slblocks.m`.
- VRFT Design app distribution: `VRFTDesignApp` is packaged as a standalone
  `build/VRFTDesignApp.mlappinstall` (self-contained, bundles the `+ddc/+vrft`
  sources) via a new programmatic `tools/packageApp.m`, and is also staged into
  the toolbox so it ships inside the `.mltbx`. `buildToolbox.m` produces both
  artifacts in one run. Build metadata (version, author) is read from
  `CHANGELOG.md` and `CITATION.cff`.
