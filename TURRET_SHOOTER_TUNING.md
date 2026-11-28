# BIOBUZZ Turret and Shooter Tuning

Tune in this order. Changing several stages at once makes an angle error look like
an RPM error. Record a flight log for every test session.

## Work completed on 2026-11-27

| Area | What changed today | Current result |
|---|---|---|
| Target selection | Kept every BIOBUZZ tag visible to the tracker, but only the correct alliance's confirmed upward CELL can become the shooting target. | Other tags help localization but can never select the hive or change the shot target. |
| Landmark assist | Added learned helper-cluster-to-target offsets when both clusters are visible together. Links expire after `30000 ms` and are rejected if roll differs by more than `12 deg`. | If target IDs disappear but a learned helper remains visible, state becomes `LANDMARK` and continues correcting the target hive. |
| Field memory | Stored the target hive as one field-space `(x, y)` point and reprojected it from the current Pinpoint pose. | With no tags, state becomes `ODOMETRY` and follows the remembered location instead of holding the last vision angle. |
| Vision policy | Kept odometry as a short bridge while requiring a recent visual correction for shot-ready. | Aim-ready expires after `250 ms` without direct-target or learned-landmark correction. |
| Velocity prediction | Increased lookahead from `80 ms` to `120 ms` and maximum lead from `6 deg` to `8 deg` after reviewing the movement lag in the log. Prediction is disabled while stopped. | Faster tracking while driving, with bounded lead and no stationary prediction jitter. |
| Source switching | Direct vision, landmark assist, and odometry now use the same fused command path. | No separate source-specific target command should create a handoff jump. |
| Removed band-aids | Removed turret pause, full-range rescan, and the old blind `current angle + 180 deg` field handoff. | The turret follows one saved field point; crossing that point naturally changes the requested bearing by about 180 degrees. |
| Feeder | Removed aim-ready/RPM gating from the driver transfer command. | Holding A always feeds; readiness remains advisory telemetry only. |
| RPM distance protection | Isolated `getShooterDistanceIn()` to the selected target CELL's filtered tag range. | Helper tags and odometry may steer the turret but cannot change shooter RPM; target loss holds the last trusted target-tag distance. |
| Shooter table | Kept/documented the current interpolation points: `95/2875`, `115/2900`, `140/2900`, and `180/3100` (`in/RPM`). | These values still need final on-field tuning after side aim is set. |
| Side aim | Added distance-correct lateral bank aiming with side-switch hysteresis/hold. | The mechanism is ready, but center/max offsets remain `0 in` until shooting tests determine safe side targets. |
| Field-crossing diagnosis | The log showed the RPM-protected distance stuck near `214 in` while live selected-target measurements were about `40-80 in`, placing the saved HIVE point outside the field. | `clean-v26-field-handoff` gives field fusion the current target-tag range while keeping RPM isolated to its protected selected-target range. |
| Stale range recovery | A single range outlier is still rejected, but three mutually agreeing outlier frames can now replace a stale RPM distance. | A real range change no longer remains rejected forever; logs emit `DISTANCE_RESEED_CONFIRMED`. |
| Reachability safety | Aim-ready now becomes false while the target is clamped at a soft stop. | The feeder remains available, but the driver is told to turn the robot toward the hive. |
| Shooter indicator | Shooter enable state now owns whether LED PWM is enabled; the turret only selects the color. | The light is fully off whenever the outtake is off in TeleOp, autonomous, and test OpModes. |
| Logging/telemetry | `bearing_deg` now reports the live fused bearing during vision loss; raw last-vision bearing is separate as `vision_bearing_deg`. Added field point and clamp values to Driver Station/Dashboard. | A new log can distinguish bad odometry, a moving anchor, aim conversion, and a mechanical clamp. |
| Verification | Compiled `:TeamCode:compileDebugJavaWithJavac`. | Build passed; only existing Java source/target deprecation warnings were reported. |

### Remaining field work

| Order | Still to do | Do not continue until |
|---:|---|---|
| 1 | Run the hidden-tag `24 in` translation, `45 deg` rotation, and field-crossing tests. | `fus_x/y` stay fixed; distance decreases then increases and bearing reverses when passing the point, with `tur_target_clamped=0`. |
| 2 | Measure and enter turret pivot forward/left offsets. | Combined translation/rotation keeps hidden-tag aim on the same point. |
| 3 | Compare prediction at `100`, `120`, and `140 ms`. | Fixed-point tracking and reachability tests pass first. |
| 4 | Tune side-aim center/max offsets in `0.25 in` steps. | Global scale/offset and camera-to-shooter geometry are correct. |
| 5 | Retune the four RPM rows and add rows only where needed. | Final side aim is chosen and target-tag distance agrees with a tape measure. |
| 6 | Run the full regression matrix at the bottom of this document. | Near/far and both-side shots are repeatable at more than one battery voltage. |

## Implemented tracking behavior

| Situation | Behavior | Required check |
|---|---|---|
| Correct upward CELL tags visible | They are the only tags allowed to choose the shooting target. | Wrong-alliance and downward CELL tags never become the target. |
| Target plus another cluster visible | Learn that cluster's field offset to the target. | `landmark_link_count` becomes greater than zero. |
| Target tags hidden, linked cluster visible | Correct target memory from the linked cluster if its roll still matches. | Tracking state becomes `LANDMARK`; target command does not jump. |
| Linked cluster changes roll or target CELL changes | Reject/clear the learned relation. | No correction from a tipped/moved stale cluster. |
| No tags visible | Project saved target with Pinpoint odometry to help reacquire. | Tracking state becomes `ODOMETRY`; aim-ready expires after `250 ms` without a visual correction. |
| Odometry invalid | Hold the last safe command and do not report ready. | Tracking state explains why fallback is unavailable. |
| Robot moving | Lead the current fused bearing using its measured angular rate. | Lead has the same sign as target motion and stays inside its clamp. |

The transfer remains driver-owned: holding A feeds even when aim-ready is false.

BIOBUZZ tags are moving landmarks. A learned link is created only when the target
and helper cluster are visible together; the code does not treat tags as fixed
absolute field coordinates.

## Odometry fixed-point and field-crossing diagnosis (2026-11-27)

The odometry math stores a field point, not a fixed robot-relative angle. The
November 27 log confirms that the point stayed fixed while the calculated angle
and distance changed with robot pose:

| Log time | Robot pose `(x, y, heading)` | Saved hive `(x, y)` | Calculated bearing | Calculated distance |
|---:|---|---|---:|---:|
| `40.428 s` | `(146.570, 26.826, -2.333 deg)` | `(261.921, 197.713)` | `-58.313 deg` | `206.175 in` |
| `42.307 s` | `(165.015, 31.955, +9.633 deg)` | `(261.921, 197.713)` | `-50.055 deg` | `192.006 in` |

Two separate problems made the behavior look like a fixed direction. First, the
cable-safe range `-100..470` ticks leaves about a `45 deg` blind sector, so an
unreachable request is intentionally clamped at a soft stop. Second, and more
important for crossing the field, the saved point above was wrong: `(261.921,
197.713)` is outside the roughly `144 x 144 in` field. The target-tag range used
to build it had remained at `214.131 in` even after current selected-target
measurements moved into roughly the `40-80 in` range.

The current code separates those jobs. Field fusion uses the current selected
target CELL range and applies field-space residual/multi-frame validation.
Shooter RPM continues to use the separately protected selected-target range.
There is no midpoint trigger and no fixed `+180 deg` command: passing the actual
saved point changes the geometry, so the requested bearing naturally rotates to
the opposite direction.

| Finding | Code/status | What the drive team should do |
|---|---|---|
| Saved hive is a real field point | `fus_x` and `fus_y` remain fixed during pure odometry | Use the translation test below to verify each deployment. |
| Saved point used a stale `214 in` range | Live target-tag range disagreed by well over `100 in` | `clean-v26-field-handoff` separates field-fusion range from RPM filtering and confirms a stale-range reseed over three frames. |
| Robot passes the physical saved point | `fus_dist_in` falls toward zero, then rises; `fus_bearing_deg` changes by about `180 deg` | Judge the handoff from position geometry, not from crossing the field midpoint. |
| Hive enters turret blind sector | `tur_target_clamped = 1`; telemetry says `UNREACHABLE` | Rotate the chassis toward the hive until it says `reachable`. Do not widen limits without checking cables. |
| Old `bearing_deg` was misleading during vision loss | It used to retain the last raw vision bearing | Current builds log live fused bearing there and keep raw vision in `vision_bearing_deg`. |
| Soft-stop position could look settled | Reaching the stop is not reaching the hive | Aim-ready is now forced false while clamped. The A-button feeder is still never gated. |
| Camera points away from all tags | Neither helper vision nor target vision can correct drift | Keep the robot generally facing the hive; odometry is only a short bridge. |

## What to do next

| Priority | Task | How to check | Pass condition |
|---:|---|---|---|
| 1 | Deploy the current build | Confirm `Turret Tune = clean-v26-field-handoff`. | Driver Station shows the expected tune version. |
| 2 | Prove fixed-point translation | Lock target with tags, cover tags, strafe `24 in` without turning. | `Hive Field Point` stays constant; bearing changes in the correct direction. |
| 3 | Prove fixed-point rotation | From the same saved point, rotate chassis about `45 deg` in place. | Field point stays constant; bearing changes about `45 deg` opposite chassis rotation. |
| 4 | Prove the position-based 180 handoff | Keep tags visible, drive toward then past the saved HIVE point, and keep the target mechanically reachable. | `fus_x/y` stays fixed, `fus_dist_in` decreases then increases, and requested bearing changes to the opposite direction. |
| 5 | Check the blind sector | Rotate until `Turret Reachability` becomes `UNREACHABLE`. | Aim-ready goes false; turning chassis toward hive makes it reachable and tracking resumes. |
| 6 | Verify range recovery | Create a real large range change and hold a stable view for at least three frames. | `vis_dist_pending_frames` counts up, `DISTANCE_RESEED_CONFIRMED` appears, and protected RPM range moves to the new target range. |
| 7 | Verify light-off behavior | Toggle the outtake off in normal and manual TeleOp. | Flywheel command is zero and the RGB indicator is completely off. |
| 8 | Keep vision available | Drive normal scoring routes with chassis generally facing the target hive. | Regular `VISION`/`LANDMARK` corrections; no long odometry-only stretch. |
| 9 | Measure turret pivot | Enter forward/left values in `OdometryConfig`. | Hidden-tag bearing stays correct during combined translation and rotation. |
| 10 | Tune prediction | Compare `100`, `120`, and `140 ms` logs after fixed-point tests pass. | Least moving lag without overshoot when stopping. |
| 11 | Tune side aim | Increase lateral offset gradually using the side-aim table. | Both sides hit safely inside the opening. |
| 12 | Retune RPM | Fill the RPM worksheet only after aim geometry is final. | Repeatable height/range at every table point. |

### Fixed-point log checklist

| Value | During hidden-tag translation | Bad result means |
|---|---|---|
| `fus_x`, `fus_y` | Constant (small floating-point variation only) | Something is reseeding/moving the field anchor. |
| `odo_x_in`, `odo_y_in` | Change by the measured robot movement | Pinpoint position/frame/configuration problem. |
| `fus_bearing_deg` / `bearing_deg` | Changes on a sideways translation | Pose-to-target projection or odometry-axis problem. |
| `fus_dist_in` | Changes on movement toward/away from hive | Pose-to-target projection or odometry-scale problem. |
| `tur_requested_target_ticks` | Follows fused bearing | Aiming conversion problem if it does not. |
| `tur_control_target_ticks` | Follows request while reachable | If different, check `tur_target_clamped` before changing aim math. |
| `tur_target_clamped` | `0` for the translation test | `1` means the mechanical blind sector invalidated the test. |
| `vis_dist_raw_in` | Follows the current selected target CELL | If sensible but `fus_x/y` is impossible, inspect camera/odometry geometry. |
| `vis_dist_pending_frames` | Usually `0`; counts consistent large range changes | Repeated resets mean the new readings do not agree within `10 in`. |

## Tuning order

| Order | Parameter or task | Current value | How to tune | Pass condition |
|---:|---|---:|---|---|
| 1 | Turret mechanical zero and limits | `-100` to `470` ticks | Center mechanically, press START once, then move carefully toward each limit. | No cable tension, hard-limit fault, or encoder-direction reversal. |
| 2 | `TICKS_PER_DEGREE` | `1.81` | Command/measure equal angles on both sides. | Measured angle scale agrees left and right. |
| 3 | `CAMERA_HEIGHT_IN` | `14.0 in` | Measure floor to lens center. | Physical measurement entered. |
| 4 | `CAMERA_UP_ANGLE_DEG` | `24.24 deg` | Measure mount pitch, then verify range at two known distances. | Near and far range errors do not have an increasing trend. |
| 5 | `CAMERA_LEFT_OF_SHOOTER_IN` | `3.5 in` | Measure camera axis left of shooter axis. | Near-range parallax is correct on both sides. |
| 6 | Turret pivot forward/left | `0.0 / 0.0 in` | Measure turret axis from the Pinpoint tracking origin. | Driving/rotating with tags hidden preserves bearing. |
| 7 | `AIM_ANGLE_SCALE` | `0.96` | Keep bank offsets zero; compare equal left/right shots. | Symmetric errors are removed. |
| 8 | `AIM_OFFSET_DEG` | `0.0 deg` | Apply only the remaining constant mechanical trim. | Center shots agree at near and far range. |
| 9 | Visual ready age | `250 ms` | Cover all tags and watch aim-ready. | Odometry keeps aiming, but ready expires after about `250 ms`. |
| 10 | Landmark tracking | roll `12 deg`, age `30000 ms` | Cover target IDs while another learned cluster remains visible. | `LANDMARK` remains stable and rejects a changed-roll cluster. |
| 11 | Prediction lookahead | `120 ms` | Compare `100`, `120`, and `140 ms` in separate logged runs. | Smallest moving-target lag without overshoot after stopping. |
| 12 | Prediction max lead | `8 deg` | Lower it if fast turns cause a large command jump. | Lead remains bounded during rapid chassis rotation. |
| 13 | Turret `POSITION_KP` | `0.040` | Tune only after prediction comparison. | Fast response without repeated overshoot. |
| 14 | Turret `VELOCITY_KD` | `0.0030` | Increase slightly only if settling oscillates. | Clean stop with no sluggish tracking. |
| 15 | `STATIC_FRICTION_POWER` | `0.025` | Use the smallest value that starts the turret reliably. | No near-target buzzing or persistent dead zone. |
| 16 | Side-aim offsets | center `0`, max `0 in` | Follow the side-aim table below. | Both shooting sides remain safely inside the CELL opening. |
| 17 | Shooter RPM table | see worksheet | Tune after final aim geometry/offsets. | Repeatable shots from each distance and both target sides. |

## Side-aim tuning

Do not use `AIM_OFFSET_DEG` to create a side shot. It is a global mechanical
trim; the same angle produces different lateral offsets at different ranges.

| Parameter | Current | Tune/check |
|---|---:|---|
| `BANK_AIM_CENTER_OFFSET_IN` | `0.0 in` | Start at zero. Increase only if a side shot is required even from the centerline. |
| `BANK_AIM_MAX_OFFSET_IN` | `0.0 in` | Increase in `0.25 in` steps. Keep the ball center inside the opening after allowing for ball radius and aim error. |
| `BANK_AIM_FULL_SIDE_IN` | `3.0 in` | Robot lateral distance from center where maximum offset is reached. |
| `BANK_AIM_SWITCH_HYSTERESIS_IN` | `3.0 in` | Increase if the chosen side chatters near center; decrease only if side switching is too late. |
| `BANK_AIM_SWITCH_HOLD_MS` | `750 ms` | Increase if rapid side flipping remains; decrease if a real center crossing reacts too slowly. |

Run each offset from robot-left, center, and robot-right at one near and one far
range. Verify the chosen side, commanded offset, impact location, and whether the
HIVE tips as intended.

## Shooter RPM worksheet

The code linearly interpolates between rows and clamps beyond the first/last
distance. Distances must remain strictly increasing and RPM must stay at or below
`MAX_RPM = 3700`.

| Distance | Current RPM | New RPM | Left-side hits / attempts | Right-side hits / attempts | Battery voltage | Notes |
|---:|---:|---:|---:|---:|---:|---|
| `95 in` | `2875` |  |  |  |  |  |
| `115 in` | `2900` |  |  |  |  |  |
| `140 in` | `2900` |  |  |  |  |  |
| `180 in` | `3100` |  |  |  |  |  |

Add rows at common shooting locations when the error between existing points is
not approximately linear. If left and right side shots consistently need different
RPM, keep the distance curve honest and add a separate small side trim instead of
putting conflicting values in one table.

Before accepting an RPM row, check:

- selected target-tag distance agrees with a tape-measured shooter-to-opening distance;
- covering the target tags while helper tags remain visible does not change RPM;
- actual RPM is stable before feeding;
- multiple balls are tested, not only one;
- both side-aim directions are tested;
- at least one run is made at a lower battery voltage;
- the impact miss is vertical/range-related, not a turret angle error.

## Final regression matrix

| Test | Pass condition |
|---|---|
| Direct target tags only | `VISION`, green LED when settled, correct CELL selected. |
| Target plus same-HIVE opposite CELL | One or more landmark links learned. |
| Cover target tags, leave a learned helper visible | `LANDMARK`, no wrong target switch, aim-ready can remain true. |
| Block all tags for more than `250 ms` | `ODOMETRY` may keep aiming, but aim-ready is false. |
| Show opponent tags | They may correct an already learned relation but never become the shooting target. |
| Change helper cluster roll | Landmark correction is rejected. |
| Block all tags and drive/turn while reachable | `fus_x/y` stay fixed, bearing/distance follow pose, and feeder still follows A directly. |
| Drive past the saved HIVE point | Distance decreases then increases and the requested bearing changes by about `180 deg`; no midfield-triggered angle jump. |
| Put hive in the turret blind sector | `tur_target_clamped=1`, aim-ready false, and telemetry tells driver to turn chassis. |
| Stop after moving | Prediction lead returns to zero and turret settles without overshoot. |
| HIVE tips | Old links clear; new upward CELL requires confirmed frames before targeting. |
| Pinpoint becomes invalid | Turret holds safely and `aimReady` is false. |
| Disable outtake in each OpMode | Outtake power is zero and `tur_led_enabled=0`; the indicator is physically off. |
| Near/far side shots | Aim point stays inside opening and RPM result is repeatable. |
