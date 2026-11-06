# Turret Tuning Guide

This guide applies to [`turretSubsystem2.java`](turretSubsystem2.java). Tune the
turret in the order below. Keep both `PREDICTION_ENABLED` and
`USE_ODOMETRY_PREDICTION` set to `false` until stationary aiming is reliable.

## BIOBUZZ AprilTag cluster targeting

BIOBUZZ uses four-tag clusters. IDs `42`, `43`, `44`, and `45` are the blue
scoring-side CELL cluster. FTC SDK 12 defines the cluster origin at the center
of the CELL opening so that it is the useful aiming point, rather than the
center of any individual printed tag.

Subsystem 2 uses the official SDK geometry:

```text
Tag ID:                 42     43     44     45
Horizontal offset:   -6.50  -2.75  +2.75  +6.50 inches
Tag strip behind aim origin: 7.1874 inches
Tag strip below aim origin: 5.622 inches
Official printed tag size:   3.25 inches / 82.55 mm
```

The red scoring cluster is IDs `30` through `33`; the blue scoring cluster is
IDs `42` through `45`. `TARGET_CLUSTER_FIRST_ID` selects between them. The live
tuner's gamepad 1 Y button switches alliance without another deployment.

When two to four tags are visible, the subsystem fits their known horizontal
positions to their measured Tx angles. This solves the cluster center and
depth without shifting toward whichever subset happens to be visible. When
only one tag is visible, it uses the Limelight 3D range or the most recent
multi-tag depth.

When that geometry solution is valid, shooter distance is derived from the
known tag spacing, average Ty, and calibrated camera pitch. The fixed tag-height
formula is used only as a fallback. This matters because the BIOBUZZ cluster is
mounted on a moving CELL and the official aim origin is not at a printed tag.

Configure the Limelight AprilTag pipeline as follows:

- Family: AprilTag Classic 36h11
- Marker size: `82.55 mm`
- ID filter: `42,43,44,45`
- Enable 3D pose calculation for reliable one-tag recovery
- Use the highest practical resolution and a fixed, low exposure

Driver Station telemetry reports the number and IDs of visible tags. `Cluster
Center: GEOMETRY SOLVED` means the corrected CELL-opening aim point is active.
`ANGULAR FALLBACK` means there was not enough geometry or 3D range data; this
normally occurs when only one tag has ever been visible and Limelight 3D is
disabled.

The old `DEFAULT_TAG_HEIGHT_IN = 14.5` is not an official BIOBUZZ CELL
measurement. The official cluster origin is at the CELL opening center, while
the stickers sit elsewhere on the moving CELL. Do not finalize shooter distance
from the height formula until `CAMERA_HEIGHT_IN`, camera pitch, and the desired
vertical aim point have been measured on the assembled robot and field element.

## Safety and encoder zero

- Remove or disable projectiles while tuning.
- Keep hands, wires, and tools outside the turret's travel.
- Begin with low manual power and be ready to stop the OpMode.
- Do not drive against a mechanical stop.

The soft limits assume encoder position `0` is the mechanical center. The
current [`Hardware.java`](../hardware/Hardware.java) captures the raw turret
encoder count during INIT and exposes a position relative to that count. Both
turret subsystems use this relative position, so encoder counts retained from a
previous OpMode can no longer falsely hold the turret at one soft limit.

**Physically center the turret before pressing INIT.** The software zero is not
a homing system and cannot know the turret's physical angle. Starting an OpMode
with the turret off-center shifts both soft limits and can allow a mechanical
collision. A competition-ready system should use a repeatable homing procedure,
limit switch, or absolute encoder.

If the turret was not centered during INIT, release auto-aim, manually place it
at the intended forward/center position, and click both gamepad stick buttons
together. `biobuzzTestTeleOp` will redefine that position as `0` and clear the
turret controller state. Only use this control at the known center position.

With the current values, each limit is approximately:

```text
135 ticks / 2.04 ticks per degree = 66.2 degrees
```

## Live tuning TeleOp

Run `Turret PID Live Tuner` from the Driver Station. Gain changes take effect in
the running OpMode immediately:

- Gamepad 1 drives; hold right trigger for auto-aim.
- Hold gamepad 1 Back and use D-pad left/right to move the turret manually.
- Click both gamepad 1 sticks only when the turret is at its known center to
  redefine encoder zero.
- Gamepad 1 D-pad up/down selects a value and left/right changes it.
- Hold gamepad 1 left bumper for one-tenth steps or right bumper for 10x steps.
- Gamepad 1 A toggles prediction, B toggles odometry blending, X toggles latency
  compensation, and Y selects the red/blue scoring cluster.

The changes live in memory, including when another OpMode is selected, until
the Robot Controller app restarts. Record the final values shown in telemetry
and copy them into `turretSubsystem2.java` to make the tune the next deployed
default.

Keep prediction, odometry blending, and latency compensation off while tuning
the stationary controller. Tune Kp and Kd first, then stiction, Ki, and Kv.

## Additional tuning telemetry

The subsystem already provides the required getters. Temporarily add this to
[`biobuzzTestTeleOp.java`](../opmodes/testing/biobuzzTestTeleOp.java):

```java
telemetry.addData("Visible IDs", turret.getVisibleTagIds());
telemetry.addData("Tx", "%.2f deg", turret.getFilteredTxDeg());
telemetry.addData("Vision error", "%.2f deg", turret.getLastVisionErrorDeg());
telemetry.addData("Ticks", "%d -> %.1f",
        turret.getCurrentTicks(), turret.getTargetTicks());
telemetry.addData("Power", "%.3f", turret.getAppliedPower());
telemetry.addData("Target rate", "%.1f deg/s",
        turret.getTargetRateDegPerSec());
telemetry.addData("Distance", "%.1f in", turret.getDistanceIn());
telemetry.addData("Latency", "%.1f ms", turret.getLastLatencyMs());
telemetry.addData("Lead", "%.2f deg / %.2f s",
        turret.getPanLeadDeg(), turret.getPredictionTimeS());
```

The test TeleOp uses D-pad left/right for manual movement and the right trigger
for auto aim. Reduce `MANUAL_TURRET_POWER` from `0.25` to approximately `0.10`
while measuring limits and directions.

## 1. Calibrate encoder scale

1. Mark the centered turret position and record the encoder position.
2. Rotate the turret through a measured angle, preferably 45 or 60 degrees.
3. Record the encoder change.
4. Calculate:

   ```text
   TICKS_PER_DEGREE = abs(change in ticks) / measured angle
   ```

5. Repeat in both directions and average the results.
6. Update `TICKS_PER_DEGREE`.

## 2. Verify direction signs

### `MOTOR_DIRECTION_SIGN`

Apply positive manual motor power and watch the encoder:

- Positive power increases ticks: use `1.0`.
- Positive power decreases ticks: use `-1.0`.

This sign is also used by the soft-stop logic.

### `AIM_DIRECTION_SIGN`

Place an allowed AprilTag 5 to 10 degrees away from the crosshair and briefly
hold auto aim:

- If the absolute vision error decreases, the sign is correct.
- If the error grows or the turret moves away, release the trigger immediately
  and flip `AIM_DIRECTION_SIGN`.

Do not tune gains until both direction tests pass.

## 3. Set safe travel limits

1. Using low manual power, move toward one safe mechanical extreme.
2. Stop before a hard stop, wire strain, or robot collision.
3. Record the encoder position.
4. Repeat at the other extreme.
5. Leave a margin of approximately 5 to 10 ticks.
6. Enter the results in `MIN_TURRET_TICKS` and `MAX_TURRET_TICKS`.
7. Verify that manual movement is stopped correctly at both limits.

## 4. Tune stationary position control

Use a stationary robot and tag. Start with:

```java
public static boolean PREDICTION_ENABLED = false;
public static boolean USE_ODOMETRY_PREDICTION = false;
public static double POSITION_KI = 0.0;
public static double VELOCITY_KV = 0.0;
public static double STICTION_POWER = 0.0;
```

The checked-in `live-tune-v2-cluster` profile uses a conservative version of the
System 1 inner loop:

```java
POSITION_KP = 0.045;
VELOCITY_KD = 0.0040;
VELOCITY_KV = 0.0015;
POSITION_KI = 0.0005;
MAX_INTEGRAL_POWER = 0.025;
STICTION_POWER = 0.050;
MAX_TURRET_POWER = 0.40;
VISION_CORRECTION_GAIN = 0.12;
```

Subsystem 2 filters encoder velocity with an alpha of `0.30` and scales the
entire controller output from 25 percent near the target to full power at a
25-tick error. For isolated P/D tuning, temporarily setting Ki, Kv, and
stiction to zero is still recommended. With prediction disabled, target-rate
feedforward is also disabled. `LATENCY_COMPENSATION_ENABLED` defaults to
`false`; enable and verify its direction only after stationary tracking is
smooth.

### Proportional gain: `POSITION_KP`

1. Start with a low value.
2. Place the target 5 to 15 degrees off-center.
3. Engage auto aim.
4. Increase Kp by roughly 20 to 25 percent per test.
5. When the turret begins to overshoot or oscillate, reduce Kp by approximately
   15 to 25 percent.

Increase Kp if response is slow. Reduce it if the turret repeatedly crosses the
target or immediately saturates at `MAX_TURRET_POWER`.

### Velocity damping: `VELOCITY_KD`

Start at zero and increase it in small increments. Increase Kd to reduce
overshoot and bouncing. Reduce it if motion becomes sluggish, noisy, or the
power chatters. Test approaches from both sides of the tag.

This controller uses velocity error rather than the mathematical derivative of
position error.

### Stiction: `STICTION_POWER`

Find the minimum manual power that reliably begins motion in each direction.
Set `STICTION_POWER` to roughly 80 to 90 percent of the larger value. For
example, if reliable motion begins at `0.035`, try `0.028` to `0.032`.

Too much stiction compensation causes twitching near the target.

### Integral gain: `POSITION_KI`

Only add integral if the turret consistently stops slightly off target under
load. Increase it in very small steps and wait at least one second during each
test. Reduce it if the turret slowly winds up and overshoots. Do not use
integral to hide an incorrect aim offset.

## 5. Tune the vision filter

`VISION_CORRECTION_GAIN` controls how quickly the encoder target follows new
vision measurements:

- Increase it if tracking visibly lags.
- Decrease it if target ticks jump or the turret follows camera noise.

The current `0.12` prioritizes smoothness. Try changes such as `0.12`, `0.16`,
and `0.20`, testing after each change.

After aiming is reliable, set `AIM_STABLE_FRAMES` to approximately 3 to 5. The
current value of 2 declares the turret ready after two acceptable vision frames.

## 6. Calibrate per-tag optical corrections

The entries in `TAG_AIM_OFFSETS_DEG` use ascending ID order within the selected
four-tag cluster. For the default blue scoring cluster:

```java
public static int TARGET_CLUSTER_FIRST_ID = 42;
```

These offsets now correct small per-tag optical or installation biases before
the cluster geometry fit. For each tag:

1. Place the robot at a fixed position where all four tags are visible.
2. Compare each tag's measured Tx against the value predicted by the known
   cluster spacing.
3. Average the residual over several frames.
4. Store the residual correction in the corresponding array position.

Example:

```java
public static final double[] TAG_AIM_OFFSETS_DEG = {
    0.35, -0.20, 0.10, 0.45
};
```

The cluster solver calculates `tag tx - correction`. Do not use these values as
a global shot offset or to conceal direction and encoder-scale errors. Apply a
separate global shot correction only after cluster-center tracking is verified.

## 7. Calibrate distance

Measure these values from the camera lens or optical center:

- `CAMERA_HEIGHT_IN`
- The center height of each tag
- The actual downward camera angle

Tag heights follow ascending ID order within the selected cluster:

```java
public static final double[] TAG_HEIGHTS_IN = {
    14.5, 14.5, 14.5, 14.5
};
```

At several tape-measured horizontal distances, compare the known distance to
`getDistanceIn()`. Estimate the camera angle at each position with:

```text
camera down angle =
    degrees(atan((camera height - tag height) / known distance)) + measured ty
```

Average several measurements and update `CAMERA_DOWN_ANGLE_DEG`.

## 8. Tune moving-target feedforward

After stationary aiming works, drive sideways slowly while prediction remains
disabled. Adjust `VELOCITY_KV`:

- If the turret consistently trails, increase Kv.
- If it gets ahead or overshoots during motion, decrease Kv.
- If it moves in the wrong direction, correct the direction/rate signs instead
  of applying an extreme gain.

Repeat at several driving speeds.

## 9. Enable prediction

Set `PREDICTION_ENABLED` to `true` only after the previous stages pass.

Measure:

- `FIXED_SHOT_DELAY_S`: firing command to projectile release
- `FIXED_FLIGHT_TIME_S`: fixed portion of flight time
- `TOF_PER_INCH_S`: change in flight time per inch

Slow-motion recordings at three or more distances can be fitted to:

```text
flight time = FIXED_FLIGHT_TIME_S + TOF_PER_INCH_S * distance
```

During lateral driving:

- A shot landing behind the moving aim point needs more prediction time.
- A shot landing ahead needs less prediction time.
- Reversed lead indicates a sign error; do not compensate with negative time.

Leave odometry prediction disabled until vision-only prediction works. Then
enable `USE_ODOMETRY_PREDICTION` and verify every odometry sign independently.

With `USE_ODOMETRY_PREDICTION` enabled, vision measurements also anchor the
CELL opening as a point in Pinpoint field coordinates. If the tags disappear,
the subsystem recomputes the target bearing and distance from the current
Pinpoint pose for up to `ODOMETRY_TARGET_HOLD_MS` (default 1000 ms). Telemetry
changes the target source from `VISION` to `ODOMETRY` during this interval.
After the timeout, the turret stops following the saved point.

This compensates for robot translation and rotation during a short occlusion.
It cannot observe motion of the BIOBUZZ CELL itself while the tags are hidden,
which is why the fallback is deliberately time-limited.

Use the live tuner's three odometry telemetry lines before enabling the blend:

1. Push the robot forward: `forward` must be positive and `left` near zero.
2. Push it left: `left` must be positive and `forward` near zero.
3. Rotate counter-clockwise: `turn` must be positive.
4. With a stationary visible target, drive in simple forward, left, and turning
   motions. The vision and odometry bearing/range rates should have the same
   sign and similar magnitude after the vision filters settle.

If a raw direction is reversed, change only its corresponding
`ODOMETRY_FORWARD_SIGN`, `ODOMETRY_LEFT_SIGN`, or `ODOMETRY_HEADING_SIGN`.
If all raw axes are correct but bearing rate is reversed, check
`ODOMETRY_BEARING_SIGN`. Do not enable odometry blending until these tests pass.

## Acceptance checks

A finished tune should:

- Approach the target correctly from both directions.
- Settle without sustained left/right oscillation.
- Hold `getLastVisionErrorDeg()` near the configured tolerance (currently
  plus or minus 0.40 degrees).
- Avoid remaining at `MAX_TURRET_POWER` during ordinary corrections.
- Stop safely at both software limits.
- Work consistently for every allowed tag, distance, and turret angle.
- Recover cleanly after briefly losing the target.
