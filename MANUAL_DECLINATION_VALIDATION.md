# Manual magnetic declination fix

## Scope

This isolated worktree is based on maintainer commit
`dc7f3f822e08b63395938ad9ec1347d88d3a1157` (`origin/main` when rebased).
Branch: `fix/manual-mag-declination`. It contains one fix commit on top of that
base. The separate `fix/omni-flight-safety` worktree and its controller/allocation
changes are untouched.

The fix was originally tested on `f116404187`, the last firmware revision
identified from the aircraft's parameter export. The EKF and world magnetic model
sources are identical between that revision and the requested maintainer base.
The optimized EKF tests were rerun after rebasing; the firmware change itself is
unchanged. The original branch tip is preserved locally as
`backup/manual-mag-declination-f116`.

Only `src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp` changes
production behavior. Validation was performed locally, without flashing firmware
or writing aircraft parameters.

## Defect and correction

With `EKF2_DECL_TYPE=0`, the old code did not select the finite manual
`EKF2_MAG_DECL` value for heading initialization or declination fusion. It could
instead use zero. When a geographic location became available, magnetic-state
resets also used the World Magnetic Model's direction independently of that
manual setting.

The fix keeps these three operations consistent in explicit manual mode:

1. Heading initialization and magnetic heading checks select the manual value,
   including zero, rather than substituting a learned or model direction.
2. Declination observations use the same manual value. Existing flags still
   determine whether attitude is updated; INIT-only mode remains INIT-only.
3. Magnetic-state resets preserve the model's field strength and inclination,
   but use the manual horizontal direction. The magnetic-bias reset uses that
   same field, avoiding an inconsistent pair of state resets.

All new branches require `mag_declination_source == 0` and a finite manual
value. Modes 1, 2, and 3 retain their existing selection/saving behavior. The
world magnetic tables, sensor health checks, parameter-save logic, gyro
propagation, estimator reset triggers, controllers, and bridge are unchanged.

This fixes selection of a manual reference; it does **not** make an interfered
compass accurate or prohibit every subsequent yaw reset. A calibration or sensor
change can still trigger the existing reset behavior. A constant declination
offset cannot correct interference that varies with location or orientation.

## Regression tests

Tests exercise the actual PX4 EKF and its sensor simulator in
`src/modules/ekf2/test/test_EKF_mag.cpp`, not a separate imitation of the math.

The focused binary contains 67 tests: the five existing magnetometer tests and
62 added cases. Tested manual angles are 0, +35, -35, +179, -179, +180, and -180
degrees. These are synthetic test inputs, not recommended aircraft settings.

Coverage includes:

- Initialization with and without a global position.
- GNSS arriving after initialization, plus subsequent model refreshes.
- Magnetic-field direction, strength, inclination, and consistent bias resets.
- Ground calibration-reset samples.
- Takeoff through the existing 1.5 m magnetic-state alignment transition.
- INIT-only tracking of +90-degree and -90-degree gyro turns, without following
  a changed compass measurement continuously.
- HEADING and AUTO/3D fusion using the manual reference.
- Non-finite manual input retaining the existing fallback behavior.
- Magnetic strength and inclination checks still rejecting unsuitable data.
- Automatic modes 1/2/3 preserving initialization, model-reset selection, and
  permission to save geographic declination.

With the final tests but **unmodified production code**, 52 cases fail and 15
pass. With the patch, all 67 pass in the optimized host build. This demonstrates
that the tests detect the original manual-declination defect.

The broader EKF suite contains 209 test cases in 26 test executables (including
the 67 focused magnetometer cases). See the generated XML/logs
under `build/px4_sitl_test/` for complete results.

Recorded results (2026-11-10):

| Build/run | Result |
| --- | --- |
| Original production code, final focused tests | 15 passed, 52 failed |
| Patched optimized build, focused tests | 67 passed, 0 failed |
| Patched optimized build, complete EKF suite | 209 tests in 26 executables passed |
| Patched UBSan build, continue-on-report | 67 focused tests and all 26 executables passed their assertions; existing warning below |
| PX4 formatting checks and `git diff --check` | Passed |

Evidence files: `mag-before.xml`, `mag-after.xml`, `ekf-tests.xml`,
`mag-sanitized.log`, `mag-sanitized.xml`, and `ekf-sanitized-tests.xml`.

## Numerical-check limitation

The UndefinedBehaviorSanitizer configuration reports an existing floating-point
zero division in `Matrix.hpp:315`: `Quaternion(AxisAngle)` normalizes an all-zero
axis before its small-angle branch replaces the result with the identity
quaternion. The first report occurs in `OutputPredictor::calculateOutputStates`
during the first IMU sample of an original test, before magnetometer processing.
Neither that code nor its callers are changed here.

An immediate-stop sanitizer run therefore cannot be reported as clean. The
continue-on-report run retains diagnostics and executes the assertions. Do not
confuse passing assertions with a warning-free sanitizer run. This separate
library issue has not been silently repaired or suppressed by the patch.

## Reproduce locally

From this worktree, configure the standard host test target:

```bash
cmake -S . -B build/px4_sitl_test -G Ninja \
  -DCONFIG=px4_sitl_test -DCMAKE_BUILD_TYPE=RelWithDebInfo
CCACHE_DISABLE=1 cmake --build build/px4_sitl_test \
  --target unit-test_EKF_mag -j 4
build/px4_sitl_test/unit-test_EKF_mag

mapfile -t ekf_targets < <(
  ninja -C build/px4_sitl_test -t targets all |
  sed -n 's/^\(unit-test_EKF_[^:]*\): CXX_EXECUTABLE_LINKER.*/\1/p'
)
CCACHE_DISABLE=1 cmake --build build/px4_sitl_test \
  --target "${ekf_targets[@]}" -j 4
ctest --test-dir build/px4_sitl_test -R '^unit-test_EKF_' --output-on-failure
```

The build downloads the repository's pinned GoogleTest dependency if absent.
`CCACHE_DISABLE=1` avoids the workstation sandbox's read-only ccache temporary
directory; it does not change estimator behavior.

## Required hardware verification before flight

Host tests are not a complete board-firmware build or flight-safety approval.
Before deployment, identify the connected Cube's exact board and current
firmware revision, preserve its firmware/parameters, and build that board target.
Do not accidentally include the other worktree's controller changes.

With propellers removed:

1. Use `EKF2_DECL_TYPE=0`, the intended finite `EKF2_MAG_DECL`, and, for
   initialization-only compass operation, `EKF2_MAG_TYPE=6`. Reboot after setting
   the mode parameters. The patch does not choose the declination for the user.
   Do not change these heading-reference settings during flight.
2. Read parameters back after reboot. Verify the initial displayed heading
   against a known physical direction, not just another potentially interfered
   magnetic sensor.
3. Start the HIL_GPS feed while stationary. Confirm that receiving the home
   location does not replace the manual parameter or unexpectedly rotate yaw.
4. Turn left and right, then return to the starting direction. Verify heading
   tracks the motion and returns appropriately. Check estimator reset counters
   and logs rather than relying on the QGC arrow alone.
5. Stop/restart the bridge and reboot the FC; repeat parameter and heading
   readback. Parameter persistence and the complete MAVLink integration require
   hardware verification; the EKF tests only verify its save-permission API.
6. Ensure Path A consumes PX4's startup heading, without applying a second
   independent declination correction. Verify real movement maps into the
   intended north/east directions before any flight test.

Retain the previously working firmware for rollback. Gyro drift, emergency yaw
resets, changing interference, and wrong initial compass readings remain possible.
