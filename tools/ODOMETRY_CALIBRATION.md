# Odometry calibration (do this before trusting odometry-aided aiming)

Why: the Pinpoint was never being configured outside Pedro's Follower, and the pose frame
was undefined at the start of every run. `Hardware` now configures + zeroes + seeds it from
`OdometryConfig` at INIT. These tests then measure the remaining scale/offset errors.

Place the robot at the start marks (see `OdometryConfig.START_*`) and keep it STILL during INIT
(IMU calibration, ~0.3 s). Run the OpMode **ODO CALIBRATION**.  Controls: Y = next test,
A = mark start, B = mark end, X = re-zero (robot still).  Telemetry shows live deltas.

| Test | What you do | What you tell me |
|---|---|---|
| STRAIGHT_FWD | Push straight along a tape measure ~96 in (longer = better). A at the start, B at the end. Do it 2-3 times. | true distance in inches |
| STRAIGHT_STRAFE | Same, pushing sideways. | true distance |
| SPIN_CCW | Spin in place with the right stick for exactly N turns (e.g. 10), ending on a floor mark. A, spin, B. | total degrees (10 turns = 3600) |
| SPIN_CW | Same, the other way. | total degrees |
| LOOP_RETURN | Drive or push a closed loop and put the robot back on its start marks. A, B. | nothing (closure error is measured) |
| HOLD_STILL | Don't touch it for 60+ s. A, wait, B. | nothing |

Then: `./tools/pull_logs.sh` and `python3 -I tools/analyze_odometry_log.py logs/<run> --truth STRAIGHT_FWD=96 SPIN_CCW=3600 SPIN_CW=3600`
which prints the exact values to paste. Re-run LOOP_RETURN and a spin afterwards to confirm.

Also measure with a tape: distance from the robot's rotation center (Pinpoint tracking point) to the
turret axis, forward and left, and put them in `OdometryConfig.TURRET_PIVOT_*`.


## How to run the tests (everything is driven or pushed by hand)

OpMode **ODO CALIBRATION**. Y = next test, A = mark start, B = mark end, X = re-zero (robot still). Left stick /
right stick X drive the robot (40% power); the drive motors float so it can also be pushed by hand.

- **STRAIGHT_FWD / STRAIGHT_STRAFE:** two tape marks 96 in apart. Line up one fixed point on the robot with the first
  mark, press A, drive or push ALL THE WAY to the second mark, let it stop, press B. Forward x3, sideways x3.
  Don't rotate the robot during the push.
- **SPIN_CCW / SPIN_CW:** press A, spin with the right stick for exactly N full turns back to a floor mark
  (10 turns = 3600 deg), let it stop, press B.
- **HOLD_STILL:** 60 s untouched.

Analyze: `python3 -I tools/analyze_odometry_log.py logs/<run> --truth STRAIGHT_FWD=96 STRAIGHT_STRAFE=96 SPIN_CCW=3600 SPIN_CW=3600`
(`--auto` also fits the pod offsets from the spin data; `--spinres CCW=+12` is available if a spin ended a few
degrees past/short of the mark instead of exactly on it.)
