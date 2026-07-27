# Tests

Unit tests live in `tests/unit`. Zephyr's
[Twister test runner](https://docs.nordicsemi.com/bundle/ncs-3.0.1/page/zephyr/develop/test/twister.html)
builds each
[Unity suite](https://github.com/nrfconnect/sdk-nrf/blob/v3.0.1/doc/nrf/test_and_optimize/test_framework/testing_unity_cmock.rst)
as a Linux executable using
[`native_sim/native/64`](https://docs.nordicsemi.com/bundle/ncs-3.0.1/page/zephyr/boards/native/native_sim/doc/index.html),
so the tests run on a development computer or GitHub Actions runner and do not
require OpenEarable hardware.
These links target the nRF Connect SDK version pinned in
[`west.yml`](../west.yml).

## GitHub Actions

The [`Unit Tests`](../.github/workflows/unit_tests.yaml) workflow is the primary
way to run the test suite. It runs for every pull request, for pushes to `main`,
and when started manually from GitHub Actions.

For pull requests, the workflow creates or updates a comment with the result.
Twister reports and logs are also available as the `unit-test-results` artifact
for 14 days.

Twister recursively discovers every `testcase.yaml` below `tests/unit`, so a
valid new suite in that directory is included automatically; the workflow does
not need to be edited. The suite must allow `native_sim/native/64` to run in
this workflow.

## Run the tests locally (optional)

Run Twister from the west workspace root: the directory containing `zephyr`,
`nrf`, and this repository. The commands below assume the repository directory
is named `open-earable-v2`; adjust the path if it is named differently.

Unity's test-runner generation requires Ruby. On Debian or Ubuntu:

```sh
sudo apt install ruby
```

Run all unit-test scenarios:

```sh
python3 zephyr/scripts/twister \
  -T open-earable-v2/tests/unit \
  -p native_sim/native/64 \
  --inline-logs
```

Run a single scenario by its name from `testcase.yaml`:

```sh
python3 zephyr/scripts/twister \
  -T open-earable-v2/tests/unit \
  -p native_sim/native/64 \
  --scenario openearable.unit.sensor_component \
  --inline-logs
```

On Windows, run the tests in WSL or a Linux container because `native_sim`
produces a Linux executable.

## Add a test suite

The existing suites demonstrate two common patterns:

- [`sensor_component`](unit/sensor_component) tests C++ production sources and
  binary serialization;
- [`ring_buffer`](unit/ring_buffer) tests a header-only C++ template.

A suite under `tests/unit/<module>/` consists of:

- `testcase.yaml`, defining a unique scenario name and allowing
  `native_sim/native/64`;
- `prj.conf`, enabling `CONFIG_UNITY` and any configuration required by the
  production code;
- `CMakeLists.txt`, passing the test source to `test_runner_generate(...)` and
  adding the production sources and include directories to the `app` target;
- test source files whose test functions start with `test_` and use Unity's
  `TEST_ASSERT_*` macros.

Production sources are not pulled in automatically. List each source and its
include directories in the suite's `CMakeLists.txt`.

The host build cannot use nRF hardware. Keep hardware-independent behavior
separate from drivers, or provide test doubles for hardware and Zephyr APIs.
The CI workflow discovers new suites automatically when they are placed below
`tests/unit`.

For C++ tests, enable `CONFIG_CPP`. Add `CONFIG_REQUIRES_FULL_LIBCPP` only when
the tested code requires the full C++ standard library. Keep the
`test_suiteTearDown` linkage adapter shown in the C++ examples so the generated
Unity runner can call the nRF Connect SDK's C teardown function.

## Generate coverage

Install `gcovr` in the active Python environment and run Twister with
[coverage enabled](https://docs.nordicsemi.com/bundle/ncs-3.0.1/page/zephyr/develop/test/coverage.html):

```sh
python3 -m pip install gcovr
python3 zephyr/scripts/twister \
  -T open-earable-v2/tests/unit \
  -p native_sim/native/64 \
  --coverage \
  --coverage-basedir open-earable-v2
```

Open `twister-out/coverage/index.html` after the run.
