# Turret telemetry workflow

1. Flash from Android Studio, run "OFFICIAL USE THIS TeleOp" (or "biobuzz testing teleop").
   Telemetry shows `Log: REC rows=... /sdcard/FIRST/biobuzz_logs/<stamp>_teleop`.
   Stop the OpMode with the normal STOP button so files are closed cleanly (a power cut still keeps all but the last ~200 ms).
2. Plug the Control Hub in over USB-C, tell Claude.
3. `./tools/pull_logs.sh` -> `logs/<timestamp>/` (loop CSV, events log, config snapshot, logcat)
4. `python3 -I tools/analyze_turret_log.py logs/<timestamp>`

Files per run: `*_loop.csv` (one row/loop, ~110 columns), `*_events.log` (AUTOAIM, TARGET_*, ROBOT_STOPPED, AIM_READY afterStop=..ms, SOFT_LIMIT, exceptions), `*_config.txt` (every tunable).
Last 40 sessions are kept on the hub.
