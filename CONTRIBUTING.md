# Contributing to the Data-Driven Control Toolbox

Thanks for your interest in contributing! This toolbox implements data-driven
and adaptive control methodologies inspired by
*An Introduction to Data-Driven Control Systems* (A. Khaki-Sedigh, Wiley, 2024)
as a MATLAB/Simulink toolbox.

> **Disclaimer:** This is an independent, unofficial project. It is not
> endorsed by or affiliated with the author or publisher of the book.

Contributions of all kinds are welcome: bug reports, feature requests,
documentation, examples, tests, and code.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
  - [Development Setup](#development-setup)
  - [Running Tests](#running-tests)
- [Reporting Issues](#reporting-issues)
- [Contributing Code](#contributing-code)
  - [Where Things Live](#where-things-live)
  - [Coding Conventions](#coding-conventions)
  - [Naming Conventions](#naming-conventions)
  - [Simulink Library Changes](#simulink-library-changes)
  - [Testing Your Changes](#testing-your-changes)
- [Pull Request Process](#pull-request-process)
- [Release Process](#release-process)

## Code of Conduct

This project is governed by the [Code of Conduct](CODE_OF_CONDUCT.md). By
participating, you agree to abide by its terms.

## Getting Started

### Development Setup

From MATLAB, with the repository root as your working directory:

```matlab
run('tools/devPath.m')   % adds toolbox/ and toolbox/lib to the MATLAB path
```

Do **not** add `toolbox/+ddc` itself to the path; only `toolbox/` should be on
the path for the `ddc.*` package to resolve correctly.

### Running Tests

```matlab
run('tests/runAllTests.m')
```

The test suite uses the `matlab.unittest` framework and is self-contained (no
external services required).

## Reporting Issues

- Before opening an issue, search existing issues to avoid duplicates.
- Include the toolbox version (`git describe --tags` or the `ToolboxVersion`),
  your MATLAB release, and OS.
- For bugs: reproduce with the smallest possible script, and share error
  messages verbatim.
- One issue per bug or feature request.

Formatting: file names and class references as `` `ddc.deepc.DeePCController` ``,
code blocks with ```matlab fences.

## Contributing Code

### Where Things Live

```
toolbox/+ddc/     MATLAB package namespace (ddc.deepc.DeePCController, ...)
toolbox/lib/      Simulink library (ddc_lib.slx) + slblocks.m
toolbox/examples/ Examples shipped inside the .mltbx
doc/              Narrative / theory documentation (not shipped)
tests/            matlab.unittest test classes + runner (not shipped)
tools/            Dev-only helpers: path setup, packaging, library build
```

### Coding Conventions

- Follow the style of the surrounding code (indentation, doc comments, error
  message wording).
- Document every public class member with a help-text comment block, following
  the pattern already used in `+ddc/*`.
- Add a comment only when it explains *why*; do not repeat what the code says.
- New shipped code must be a `matlab.System` class when it is stateful, so it
  works both in Simulink and plain MATLAB.

### Naming Conventions

- Package reference: `ddc.<category>.<Name>`, e.g. `ddc.deepc.DeePCController`.
- Simulink library files/subsystems prefixed `ddc_*`.

### Simulink Library Changes

The library contents are defined by `tools/buildLibraries.m` (the source of
truth) and materialized in `toolbox/lib/ddc_lib.slx`. After adding or removing
blocks:

```matlab
run('tools/buildLibraries.m')   % regenerates ddc_lib.slx
```

If you hand-edit in the Simulink editor and save, keep the two consistent.

### Testing Your Changes

Every new or modified feature should be covered by a test in `tests/`:

- Add a new test class file (e.g. `tests/tMyFeature.m`) using
  `matlab.unittest.TestCase`, or extend an existing one where it belongs.
- Run `tests/runAllTests.m` and confirm all tests pass before submitting.

## Pull Request Process

1. Fork the repository and create a branch off `main`:
   `git checkout -b fix/descriptive-name`.
2. Make your changes, following the conventions above.
3. Add or update tests; run the full suite.
4. Rebuild `ddc_lib.slx` if the Simulink library changed.
5. Update `CHANGELOG.md` under `[Unreleased]`, and README only if user-facing
   behavior changed.
6. Open a pull request. The title should summarize the change; the description
   should state the problem, the approach, and how it was tested.
7. Keep the PR focused: one logical change per PR.

Reviewers may request changes. Please update your branch rather than force
pushing over review feedback, and reply to review comments.

## Release Process

Maintainers handle releases as tagged semantic versions on `main`, e.g.
`v0.1.0`, with the `.mltbx` attached to the GitHub Release.