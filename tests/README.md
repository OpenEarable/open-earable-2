# Testing

OpenEarable uses the Unity and CMock integration supplied by nRF Connect SDK,
with Zephyr's Twister test runner. This keeps the tests compatible with the SDK
version pinned in `west.yml` and provides one workflow for unit tests, simulated
integration tests, on-target tests, and coverage.

## Run the unit tests

Run these commands from the root of the west workspace (the directory that
contains `zephyr`, `nrf`, and this repository):

Unity's generated runner requires Ruby. On Debian/Ubuntu, install the host
dependency with `sudo apt install ruby`.

```sh
python3 zephyr/scripts/twister \
  -T open-earable-v2/tests/unit \
  -p native_sim/native/64 \
  --inline-logs
```

Replace `open-earable-v2` if the application directory has a different name.
The `native_sim` target builds a native Linux executable and does not require an
nRF5340 or other target hardware. It is suitable for local TDD. Run it in WSL or
a Linux container on Windows; GitHub Actions provides a Linux environment for
every pull request.

The dedicated `Unit Tests` GitHub Actions workflow runs the same command for
pull requests and pushes to `main`. Its `Unity tests (native_sim 64-bit)` job is
the check to select when configuring branch protection. The workflow creates or
updates one pull-request comment with the result and Unity failure diagnostics.
Twister XML, JSON, and log files are retained as the `unit-test-results`
artifact for 14 days.

To run one scenario while developing:

```sh
python3 zephyr/scripts/twister \
  -T open-earable-v2/tests/unit \
  -p native_sim/native/64 \
  --scenario openearable.unit.sensor_component \
  --inline-logs
```

## Generate branch coverage

Install `gcovr` in the active Python environment, then add Twister's coverage
option:

```sh
python3 -m pip install gcovr
python3 zephyr/scripts/twister \
  -T open-earable-v2/tests/unit \
  -p native_sim/native/64 \
  --coverage \
  --coverage-basedir open-earable-v2
```

Open `twister-out/coverage/index.html`. The HTML report includes line and branch
coverage. Coverage is a development aid rather than a repository-wide gate:
new, self-contained classes can target full branch coverage without forcing
hardware-dependent firmware paths into host unit tests.

## Add a test

Create a directory under `tests/unit/<module>/` with:

- `CMakeLists.txt` using `test_runner_generate(...)` and adding the test plus
  production sources to `app`;
- `prj.conf` with `CONFIG_UNITY=y` (and `CONFIG_CPP=y` plus
  `CONFIG_REQUIRES_FULL_LIBCPP=y` for production code using the C++ standard
  library);
- `testcase.yaml` allowing `native_sim/native/64`; and
- test functions prefixed with `test_` and Unity's `TEST_ASSERT_*` macros.

Keep business logic independent from Zephyr drivers where practical. For a
module that calls hardware or Zephyr APIs, generate mocks with CMock's
`cmock_handle(...)`. Ztest remains available for tests that specifically benefit
from Zephyr test fixtures or other RTOS-aware scaffolding.

Every bug fix should add a test that fails before the fix and passes after it.
Name the test after the observable behavior, and mention the issue or regression
in a short comment when the scenario is not self-explanatory.
