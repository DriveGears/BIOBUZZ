# BIOBUZZ Shooter and Turret TODO

This file is the bring-up and calibration checklist for:

- `shooterSubsystem.java`
- `turretSubsystem.java`
- `../../opmodes/testing/biobuzzTeleOp.java`

Values currently in the Java files are starting values. Do not treat values copied from
`fromOldRobot/TurretTeleOp.java` as calibrated for this robot.

## Current priority: measure distance versus RPM

Ignore auto-aim, AprilTags, odometry, and turret PID tuning for this test. The goal right now is
only to replace `SHOT_DISTANCE_IN` and `SHOT_RPM` with repeatable measurements from the current
shooter.

### Before shooting

- [ ] Keep the robot stationary and manually point the shooter at the center of the real practice
      cell opening. Do not hold right trigger; auto-aim is not needed.
- [ ] Keep the launch angle, hood position, compression, and wheel setup unchanged for every test.
- [ ] Measure horizontal distance consistently from the shooter exit to the same reference plane on
      the cell. Do not mix camera-to-tag distance with shooter-to-cell distance.
- [ ] Confirm telemetry's actual RPM with an optical tachometer if one is available.
- [ ] Confirm actual RPM settles near target RPM before collecting trajectory data. If it cannot
      hold the target within roughly `READY_TOLERANCE_RPM`, fix RPM conversion/PIDF first.

### Important: RPM is not ticks per second

The old robot's lookup table was in encoder ticks per second. `SHOT_RPM` must contain actual
flywheel RPM. For a 28-tick encoder and direct drive:

```text
RPM = ticksPerSecond * 60 / 28
700 TPS = 1500 RPM
1250 TPS = about 2679 RPM
```

Do not paste old values such as `700` or `1250` directly into `SHOT_RPM` unless a tachometer proves
that those are the desired physical flywheel RPMs. The subsystem converts `SHOT_RPM` back to motor
ticks per second when commanding the motor.

### Test procedure

1. Start **BioBuzz Auto Aim Test** with no game element touching the feeder.
2. Press `X` once to enter manual RPM mode. Telemetry should say `rpm=MANUAL`.
3. Press `A` to enable the flywheel. Note that the current test OpMode also runs the intake and
   transfer when `A` is enabled.
4. Use D-pad up/down to change the target in 25 RPM steps.
5. Wait until telemetry reports `Flywheel ready` before each shot.
6. Feed one element through the normal intake/transfer mechanism. Keep hands and loose objects away
   from the spinning flywheel and feeder.
7. Start below the expected speed, increase by 100 RPM until shots reach the cell, then use 25 RPM
   steps to center the landing point in the opening.
8. Shoot at least 5-10 elements at that setting. Record target RPM, actual RPM before the shot,
   lowest RPM after the shot, and makes/attempts.
9. Use the lowest setting that scores reliably, then add a small competition margin (usually
   50-100 RPM, verified by testing).
10. Repeat at useful field distances. A simple starting set is every 12 inches across the robot's
    expected shooting range; add extra points only where the required RPM changes sharply.
11. Repeat the test separately for POLLEN and NECTAR. Keep separate tables if one table is not
    reliable for both element sizes.

Suggested paper/spreadsheet log:

| Element | Distance (in) | Target RPM | Actual before | Lowest after | Makes / attempts | Notes |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| POLLEN |  |  |  |  |  / 10 |  |
| NECTAR |  |  |  |  |  / 10 |  |

### Put the results into code

Keep the arrays paired and sorted by increasing distance:

```java
public static final double[] SHOT_DISTANCE_IN = {
        // measured distances
};
public static final double[] SHOT_RPM = {
        // measured RPM at each matching distance
};
```

The subsystem linearly interpolates between measurements. Test the interpolated midpoints after
entering the table. Do not enable automatic distance mode until the camera/odometry distance is
known to represent the same shooter-to-cell distance used during calibration.

If the combined `A` control makes controlled testing difficult, the next small code change should
separate **flywheel spin** from **feed one element** in the testing OpMode. That does not require any
turret work.

## Safe bring-up order

- [ ] Mechanically center the turret before initializing the OpMode. `Hardware` resets the
      turret encoder during initialization, so encoder zero and the soft limits are only valid
      if the turret starts centered.
- [ ] Verify turret encoder direction and motor direction at low power.
- [ ] Measure turret ticks per degree and set the soft limits.
- [ ] Tune stationary turret control with prediction and odometry disabled.
- [ ] Verify flywheel encoder conversion and tune its velocity PIDF.
- [ ] Create new BIOBUZZ shot tables using measured distances and RPMs.
- [ ] Configure BIOBUZZ cell-cluster targeting.
- [ ] Configure and validate Pinpoint before enabling motion prediction or tag-loss tracking.

## Test OpMode controls

Use **BioBuzz Auto Aim Test** (`biobuzzTeleOp.java`):

- Left/right sticks: drive
- Hold right trigger: auto-aim
- D-pad left/right: manual turret
- `A`: toggle shooter, intake, and transfer
- `B`: stop shooter, intake, and transfer
- `X`: switch between distance-table RPM and manual RPM
- In manual RPM mode, D-pad up/down: change target by 25 RPM

Telemetry displays the selected tag, vision error, distance, turret target/power, and
target/commanded/actual flywheel RPM.

## Turret calibration

### 1. Measure encoder scale

1. Center and mark the turret.
2. Record its encoder position.
3. Rotate it through a measured angle, preferably 90 degrees or more.
4. Calculate:

   ```text
   ticksPerDegree = abs(endTicks - startTicks) / measuredDegrees
   ```

5. Repeat clockwise and counterclockwise several times and average the results.
6. Put the result in `TICKS_PER_DEGREE`.

Do not keep both the old `2.04` value and the current `1.81` value in different parts of the
system. Every encoder/angle conversion must use the same measured value.

### 2. Verify signs and limits

- If auto-aim corrects away from the target, flip `AIM_DIRECTION_SIGN`.
- If the motor power direction and encoder direction disagree, flip `MOTOR_DIRECTION_SIGN`.
- Measure the safe physical travel from center and update `MIN_TURRET_TICKS` and
  `MAX_TURRET_TICKS` with a small safety margin.
- A homing switch or absolute encoder should eventually replace the assumption that the turret
  is centered at initialization.

### 3. Tune the controller

Keep `PREDICTION_ENABLED` and `USE_ODOMETRY_PREDICTION` false while tuning.

1. Start with `VELOCITY_KD`, `VELOCITY_KV`, `POSITION_KI`, and `STICTION_POWER` at zero.
2. Increase `POSITION_KP` until the turret responds promptly but does not oscillate. If it begins
   oscillating, reduce P by roughly 20-30%.
3. Add only enough `STICTION_POWER` to start small moves reliably.
4. Add `VELOCITY_KD` to damp overshoot.
5. Add `VELOCITY_KV` only after stationary aiming works; it is feed-forward for a moving target.
6. Use `POSITION_KI` last, and only for a small repeatable residual error. Fix camera alignment,
   backlash, and friction before using integral to hide those problems.
7. Tune `VISION_CORRECTION_GAIN` after the motor loop works. A smaller value is smoother; a
   larger value responds faster but passes more camera noise into the setpoint.

Do all initial tuning with the chassis stopped and a stationary target. Enable latency and motion
prediction only after this test is repeatable.

## Flywheel RPM calibration

### 1. Verify the RPM conversion

- `MOTOR_TICKS_PER_REVOLUTION` must describe the encoder on the motor shaft. It is commonly 28,
  but verify the actual motor.
- For a direct drive, `MOTOR_REVS_PER_FLYWHEEL_REV` is `1.0`.
- For a geared/belt drive:

  ```text
  motorRevsPerFlywheelRev = flywheelPulleyTeeth / motorPulleyTeeth
  ```

- Compare telemetry's actual RPM with an optical tachometer before tuning anything else.

### 2. Tune velocity PIDF before the shot table

1. Run the wheel without feeding game elements and measure maximum ticks/second.
2. A useful initial REV Hub feed-forward estimate is:

   ```text
   F = 32767 / measuredMaximumTicksPerSecond
   ```

3. Begin with I and D at zero, then increase P until speed recovery is quick without oscillation.
4. Add a small D value only if the response overshoots.
5. Add I only if a persistent steady-state error remains; it is often unnecessary.
6. Test recovery after feeding individual game elements, not just the unloaded wheel.

### 3. Build the distance/RPM table

1. Place the shooter at accurately measured distances from the **cell opening aim point**.
2. Select manual RPM mode with `X` and adjust RPM with D-pad up/down.
3. Wait for `Flywheel ready`, then shoot 5-10 elements at each distance.
4. Record the lowest RPM that scores consistently and add a small competition margin.
5. Replace the paired `SHOT_DISTANCE_IN` and `SHOT_RPM` arrays with the measured data.
6. Keep distances in increasing order; the subsystem interpolates between adjacent entries.
7. Test POLLEN and NECTAR separately. Their sizes differ, so separate tables or a driver-selected
   element mode may be needed if they do not leave the shooter with the same trajectory.

Do not finalize the table using the current fixed tag-height distance calculation. BIOBUZZ tags
are mounted under a moving cell; use the cluster's virtual cell-opening pose/POI instead.

## BIOBUZZ AprilTag layout

Each cell has one four-tag cluster. The official IDs are:

| Alliance | Cell location | Tag IDs |
| --- | --- | --- |
| Red | Opposite audience | 30, 31, 32, 33 |
| Red | Audience side | 34, 35, 36, 37 |
| Blue | Audience side | 38, 39, 40, 41 |
| Blue | Opposite audience | 42, 43, 44, 45 |

`TARGET_TAG_IDS = {42, 43, 44, 45}` therefore recognizes only one blue cell, not the whole blue
hive. Alliance selection must accept both four-tag clusters for that alliance.

### Target-selection TODO

- [ ] Treat each group of four IDs as one cell cluster.
- [ ] Reject clusters belonging to the other alliance.
- [ ] Select the right-side-up/scorable cell using cluster orientation. The official FTC rule of
      thumb is `abs(roll) < 90 degrees` for the right-side-up cluster.
- [ ] Require several stable frames and switching hysteresis so the selected cell does not chatter.
- [ ] Prefer pose confidence/tag visibility or target area as a secondary quality test.
- [ ] Do not aim at whichever individual tag is highest or nearest image center.

The FTC SDK 12 cluster detector reports one pose per cluster and places that virtual pose near the
center of the cell opening. Because this robot uses a Limelight, configure equivalent behavior:

- [ ] Update the Limelight to an OS version that supports per-tag Points of Interest.
- [ ] Upload a BIOBUZZ field map containing tags 30-45.
- [ ] Assign every tag a per-tag POI at the center of its own cell opening.
- [ ] Enable **Use Map POI Offsets**.
- [ ] Aim and range from the active POI rather than raw individual-tag `tx`, `ty`, or tag height.
- [ ] Validate Limelight pose-axis and roll conventions on the real practice hive before relying on
      orientation selection.

References:

- [Official BIOBUZZ Competition Manual](https://ftc-resources.firstinspires.org/ftc/game/cm-html)
- [FTC AprilTag Clusters guide](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html)
- [Limelight 3D AprilTag/POI guide](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d)

## Aiming when tags leave the camera view

The current implementation holds the last turret setpoint for only 350 ms. It reads Pinpoint
velocity for optional shot lead, but it does not yet remember a field-coordinate cell opening.

Future tracking should store the last verified opening pose in field coordinates. On every loop:

```text
dx = openingFieldX - shooterFieldX
dy = openingFieldY - shooterFieldY

distance = hypot(dx, dy)
fieldBearing = atan2(dy, dx)
robotRelativeBearing = wrap(fieldBearing - robotHeading)
turretTargetTicks = robotRelativeBearing * ticksPerDegree
```

Account for the shooter's offset from the robot tracking center before calculating `dx` and `dy`.

### Odometry tracking TODO

- [ ] Configure Pinpoint pod offsets, encoder resolution, encoder directions, starting pose, and
      heading in one place during hardware initialization.
- [ ] Verify forward, left, and turning signs using telemetry.
- [ ] When a valid cluster is visible, update/correct the stored robot and opening pose.
- [ ] When it is temporarily hidden, compute bearing and range from Pinpoint pose.
- [ ] Expire blind aiming after a time/distance/heading uncertainty threshold.
- [ ] Do not report `readyToShoot` when the remembered target is too old or localization is invalid.
- [ ] Invalidate or reacquire after a possible HIVE TIP: the remembered cell can become the
      downward, unscorable cell while it is out of view.
- [ ] Only then enable `USE_ODOMETRY_PREDICTION` and `PREDICTION_ENABLED`.

## Competition validation

- [ ] Test under bright and dim field lighting.
- [ ] Test with one, two, three, and four visible tags from a cluster.
- [ ] Test while the other cell's cluster is also visible.
- [ ] Test immediately before and after a HIVE TIP.
- [ ] Test at both turret soft limits.
- [ ] Test while driving toward, away from, and sideways relative to the hive.
- [ ] Log requested RPM, actual RPM, distance, tag/cluster selection, turret error, battery voltage,
      and whether the shot scored.
- [ ] Keep prediction disabled unless it produces a measured improvement over stationary aiming.
