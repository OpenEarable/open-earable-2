# Contributing

This repository contains the OpenEarable 2 firmware for the `openearable_v2/nrf5340/cpuapp` target. Contributions should match the current nRF Connect SDK and Zephyr-based build, keep the codebase maintainable, and leave enough documentation for the next contributor to extend the work safely.

## Core Principles

- Keep changes small, focused, and easy to review.
- Prefer extending the existing module boundaries over adding new cross-cutting abstractions.
- Preserve a linear Git history by rebasing instead of merging `main` into feature branches.
- Use conventional commits so history stays searchable and automation-friendly.
- Document architecture, public APIs, and non-obvious logic as part of the change.

## Repository Overview

The main firmware lives at the repository root and is built through Zephyr and the nRF Connect SDK.

- `src/audio`, `src/bluetooth`, `src/modules`: runtime audio, Bluetooth, and application modules.
- `src/SensorManager`, `src/Battery`, `src/SD_Card`, `src/time_sync`: sensor, power, storage, and synchronization subsystems.
- `src/drivers`, `src/Wire`, `src/utils`, `src/buttons`, `src/ParseInfo`: reusable device support and shared utilities.
- `boards/`, `dts/`: board overlays and devicetree bindings for the OpenEarable hardware.
- `tools/flash`, `tools/buildprog`, `tools/uart_terminal`: local developer tooling for flashing and diagnostics.
- `.github/workflows/`: CI builds and release automation.

Before introducing a new directory or abstraction, verify that an existing subsystem is not already the correct extension point.

## Development Setup

Use the same toolchain versions the repository expects.

1. Install Visual Studio Code and the nRF Connect for VS Code extension.
2. Install the J-Link Software and Documentation Package.
3. Install `nrfutil` and ensure it is available on your `PATH`.
4. Install nRF Connect SDK `v3.0.1`.
5. Install toolchain `v3.0.1`.
6. Open this repository as an application in the nRF Connect extension.

The manifest in [west.yml](west.yml) pins the workspace to `sdk-nrf` `v3.0.1`. Keep documentation and local validation aligned with that version unless the repository is explicitly upgraded.

## Build And Flash

For local development, the primary target is `openearable_v2/nrf5340/cpuapp`.

### Recommended VS Code Build

- Use the nRF Connect extension to add a build configuration for `openearable_v2/nrf5340/cpuapp`.
- For FOTA builds, set `-DFILE_SUFFIX="fota"` and use a build directory such as `build_fota`.
- For non-FOTA builds, use the standard project configuration without the FOTA suffix.

### Command-Line Build

When working from a Zephyr workspace, mirror the CI build:

```bash
west build --board openearable_v2/nrf5340/cpuapp --pristine=always . -- -DFILE_SUFFIX="fota"
```

If you are building from a workspace where this repository is checked out as an application directory, point `west build` at the repository path instead of `.`.

### Flashing And Recovery

- Use [tools/flash/flash_fota.sh](tools/flash/flash_fota.sh) to flash a FOTA build with the correct left/right and hardware configuration flags.
- Use [tools/flash/recover.sh](tools/flash/recover.sh) if the board needs a full recover before reflashing.
- Keep the device powered through USB or a sufficiently charged battery during flashing and recovery.

Update this guide when the build directory names, flashing scripts, board targets, or required tool versions change.

## Branching And Commit Workflow

- Create a dedicated branch from the latest `main`.
- Rebase regularly onto `main`.
- Do not merge `main` into your branch.
- When pushing rebased history, use `git push --force-with-lease`.

Recommended workflow:

```bash
git checkout main
git pull --rebase origin main
git checkout -b <topic-branch>
```

Before opening or updating a pull request:

```bash
git checkout main
git pull --rebase origin main
git checkout <topic-branch>
git rebase main
```

## Conventional Commits

All commits must follow the [Conventional Commits](https://www.conventionalcommits.org/) format:

```text
<type>(<scope>): <short summary>
```

Examples:

```text
feat(sensor-manager): add runtime sampling guard for PPG
fix(flash): validate hardware version before writing UICR
docs(contributing): align contributor guide with Zephyr firmware workflow
refactor(bluetooth): split stream setup from connection state handling
test(ci): add build coverage for release headset configuration
```

Allowed types:

- `feat`
- `fix`
- `refactor`
- `docs`
- `test`
- `chore`
- `build`
- `ci`
- `perf`

## Code Quality Expectations

### Architecture

- Keep module responsibilities explicit and cohesive.
- Avoid mixing hardware access, business logic, and transport logic in the same change unless the behavior truly spans those layers.
- Reuse the existing subsystem layout in `src/` before introducing a new top-level concept.
- Remove dead code and stale indirection when touching a relevant area.
- If a new abstraction is necessary, document why the existing structure is not sufficient.

### Documentation

This repository expects code to be documented.

- Add documentation comments to public classes, public functions, public headers, and reusable module entry points.
- Document parameters, return values, ownership rules, side effects, failure modes, and hardware assumptions when they are not obvious from the signature.
- Add brief intent comments above complex or safety-critical logic blocks where structure alone is insufficient.
- Keep comments synchronized with the implementation.
- Update repository documentation when changing developer workflows, build behavior, repository structure, or subsystem responsibilities.

### Style

- Follow the existing naming and file layout conventions.
- Prefer clear control flow over compact but opaque code.
- Avoid unrelated formatting churn.
- Keep headers and source files aligned: declarations, ownership, and invariants should be easy to trace.

## Validation Before Opening A Pull Request

At minimum, contributors should validate that the firmware still builds with the repository's supported configuration.

### Required

Run a clean FOTA build equivalent to CI:

```bash
west build --board openearable_v2/nrf5340/cpuapp --pristine=always . -- -DFILE_SUFFIX="fota"
```

### Recommended When Relevant

- Rebuild any additional configuration affected by the change.
- Flash hardware and smoke-test the changed behavior when the work touches sensors, Bluetooth, power management, storage, or audio paths.
- Verify any developer tooling changes with the corresponding script in `tools/`.

If a change cannot be validated locally, explain the gap in the pull request.

## Pull Request Guidelines

- Use a clear title that matches the final change.
- Describe the problem, the chosen solution, and relevant tradeoffs.
- Call out hardware dependencies, risky paths, and follow-up work explicitly.
- Include logs, screenshots, or recordings when they help review tooling or workflow changes.
- Keep each pull request scoped tightly enough for a focused review.

Before requesting review, confirm that:

- your branch is rebased onto the latest `main`
- commits follow conventional commit rules
- public APIs and changed behavior are documented
- the relevant firmware build succeeds
- repository documentation is updated where needed

## What To Avoid

- Merging `main` into feature branches
- Force-pushing with `--force`
- Bundling unrelated cleanup into functional changes
- Adding undocumented public APIs or hardware assumptions
- Changing build or flash behavior without updating the docs and scripts together
- Opening a pull request without validating the affected configuration

## Questions And Ambiguity

When the correct approach is unclear:

- prefer the simpler design
- document assumptions in the pull request
- ask for clarification before making a large or irreversible change

Clean history, accurate documentation, and maintainable firmware structure are part of the contribution, not optional follow-up work.
