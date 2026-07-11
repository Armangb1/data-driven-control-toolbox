# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

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
- Six per-algorithm Simulink libraries (`ddc_common_lib`, `ddc_deepc_lib`, `ddc_mfac_lib`,
  `ddc_ufc_lib`, `ddc_spsa_lib`, `ddc_str_lib`), grouped under one "Data-Driven Control
  Toolbox" node in the Simulink Library Browser via `toolbox/slblocks.m`.
