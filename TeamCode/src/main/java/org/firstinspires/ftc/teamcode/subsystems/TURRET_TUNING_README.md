# Turret Setup and TeleOp Integration

The turret gains and limits are now fixed in
[`turretSubsystem2.java`](turretSubsystem2.java). The separate PID tuning
OpMode has been removed.

## Before INIT

- Physically center the turret before pressing INIT. INIT records that position
  as encoder zero; this is not automatic homing.
- Keep the turret clear of wires and mechanical stops.
- The blue target cluster is AprilTag IDs 42-45. For red, set
  `TARGET_CLUSTER_FIRST_ID` to `30` before constructing the subsystem.

## Turret tuning

The live tuning OpMode is no longer needed. If the mechanism changes, edit the
constants at the top of `turretSubsystem2.java`, rebuild, and test one stage at
a time. Remove projectiles while tuning and be ready to stop the OpMode.

### 1. Encoder scale and directions

1. Mark the centered position and record the encoder count.
2. Manually rotate the turret through a measured angle, preferably 45 or 60
   degrees.
3. Calculate `TICKS_PER_DEGREE = abs(tick change) / measured angle`.
4. Repeat in both directions and average the results.
5. Apply positive manual power. If encoder ticks decrease, change
   `MOTOR_DIRECTION_SIGN`.
6. Place a tag slightly off-center and briefly enable auto-aim. If the turret
   moves away from it, immediately stop and change `AIM_DIRECTION_SIGN`.

### 2. Soft limits

Using low manual power, approach each safe travel extreme without touching a
hard stop. Leave a 5-10 tick safety margin and enter the positions as
`MIN_TURRET_TICKS` and `MAX_TURRET_TICKS`. Verify that manual power cannot move
past either limit.

### 3. PID tuning

The turret uses a software PID-style position controller. Its main output is:

```text
power = Kp * position error
      + Kd * velocity error
      + Ki accumulated position error
      + Kv * target velocity
      + stiction compensation
```

`VELOCITY_KD` acts on target velocity minus measured motor velocity, so it is a
velocity-damping term rather than a raw derivative of position error. The
integral contribution is limited by `MAX_INTEGRAL_POWER` to reduce windup.

The checked-in values are:

```text
POSITION_KP         0.045
VELOCITY_KD         0.0040
POSITION_KI         0.0005
VELOCITY_KV         0.0015
MAX_INTEGRAL_POWER  0.025
STICTION_POWER      0.050
MAX_TURRET_POWER    0.40
```

Keep `PREDICTION_ENABLED`, `USE_ODOMETRY_PREDICTION`, and
`LATENCY_COMPENSATION_ENABLED` off while tuning on a stationary robot and tag.
If retuning from scratch, temporarily set `POSITION_KI`, `VELOCITY_KV`, and
`STICTION_POWER` to zero.

Tune in this order:

1. Start with low `POSITION_KP`. Increase it until the turret responds quickly.
   When it begins to overshoot or oscillate, reduce it about 15-25%.
2. Start `VELOCITY_KD` at zero and increase it in small steps until overshoot
   settles. Reduce it if movement becomes sluggish, noisy, or chatters.
3. Find the lowest manual power that reliably starts movement in both
   directions. Set `STICTION_POWER` to roughly 80-90% of the larger value.
4. Add `POSITION_KI` only if the turret consistently stops slightly off target
   under load. Increase it very slowly; reduce it if the turret winds up and
   crosses the target after a delay.
5. Keep `VELOCITY_KV` at zero for stationary tests. Tune it later while driving
   to correct consistent lag during predicted movement.
6. Finally adjust `VISION_CORRECTION_GAIN`: increase it if target tracking lags,
   or decrease it if the setpoint follows camera noise.

Change one value at a time and approach the target from both directions. Do not
use integral, stiction, or a large optical offset to hide an incorrect encoder
scale or direction sign.

### 4. Moving-target prediction

Only enable `PREDICTION_ENABLED` after stationary aiming is reliable.

- `FIXED_SHOT_DELAY_S` is command-to-projectile-release delay.
- `FIXED_FLIGHT_TIME_S` is the fixed portion of flight time.
- `TOF_PER_INCH_S` is the additional flight time per inch.
- If the turret trails a moving target, carefully increase prediction time or
  `VELOCITY_KV`. If it leads or overshoots, decrease it.

Use slow-motion video at several distances to measure flight time. A reversed
lead is a direction problem, not a reason to use negative prediction time.

### 5. Odometry tracking

Enable `USE_ODOMETRY_PREDICTION` only after vision-only tracking works. Verify:

- Moving the robot forward produces positive forward velocity.
- Moving left produces positive left velocity.
- Counter-clockwise rotation produces positive heading velocity.
- Vision and odometry bearing/range rates have matching signs while driving.

Correct only the corresponding `ODOMETRY_*_SIGN` when an axis is reversed.
During a final test, cover the tags briefly and confirm
`isOdometryTrackingActive()` becomes true and the turret continues in the
correct direction.

### Acceptance checks

- The turret approaches correctly and settles from both directions.
- It stays within `AIM_TOLERANCE_DEG` without sustained oscillation.
- Normal corrections do not remain pinned at `MAX_TURRET_POWER`.
- Both soft limits stop movement before the mechanical limits.
- Tracking recovers cleanly after a brief loss of vision.

## Add auto-aim to another TeleOp

Create the subsystem once after creating `Hardware`:

```java
turretSubsystem2 turret = new turretSubsystem2(robot);
ElapsedTime loopTimer = new ElapsedTime();
```

Reset the timer after `waitForStart()`. Then call the combined update exactly
once per loop:

```java
double dtSeconds = loopTimer.seconds();
loopTimer.reset();

boolean autoAim = gamepad1.right_trigger > 0.20;
double manualPower = ((gamepad1.dpad_right ? 1.0 : 0.0)
        - (gamepad1.dpad_left ? 1.0 : 0.0)) * 0.25;

turret.update(dtSeconds, autoAim, manualPower);
```

Call `turret.stop()` when the OpMode exits. The combined update handles the
Limelight, turret controller, Pinpoint update, odometry rates, and short
vision-loss tracking; the other TeleOp should not duplicate that turret logic.

`USE_ODOMETRY_PREDICTION` remains off by default for safety. Turn it on only
after confirming that Pinpoint forward, left, and heading signs are correct.
When enabled, the subsystem saves the last visible target in field coordinates
and continues tracking it for up to `ODOMETRY_TARGET_HOLD_MS` after vision is
lost.

## Shooter RPM

Auto-aim integration does not require an RPM change. Use the turret's distance
to select the existing calibrated RPM:

```java
if (turret.hasDistance()) {
    shooter.setTargetForDistance(turret.getImpactDistanceIn());
}
shooter.update(dtSeconds);
```

Only change `SHOT_DISTANCE_IN` / `SHOT_RPM` in `shooterSubsystem.java` if
on-field shots consistently land high or low at a measured distance. Check
`MOTOR_TICKS_PER_REVOLUTION` and `MOTOR_REVS_PER_OUTTAKE_REV` first if reported
RPM does not match a tachometer.

## Quick checks

- Hold auto-aim and confirm the turret moves toward the target.
- Confirm the soft limits stop motion at `MIN_TURRET_TICKS` and
  `MAX_TURRET_TICKS`.
- Confirm `hasDistance()` becomes true before using distance-based RPM.
- If odometry tracking is enabled, briefly cover the tags while driving and
  confirm `isOdometryTrackingActive()` becomes true without reversing aim.
