package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.OdometryConfig;
import org.firstinspires.ftc.teamcode.subsystems.FlightRecorder;

import java.util.Locale;

/**
 * Odometry calibration + accuracy logger. See tools/ODOMETRY_CALIBRATION.md.
 *
 *   Y  : cycle test type          A : mark START          B : mark END
 *   X  : re-zero IMU + pose (robot must be still)
 *   Left stick / right stick X : drive (mecanum, 40%)
 */
@TeleOp(name = "ODO CALIBRATION", group = "TeleOp")
public class OdometryCalibration extends LinearOpMode {

    private static final String[] TESTS = {
            "STRAIGHT_FWD", "STRAIGHT_STRAFE", "SPIN_CCW", "SPIN_CW", "LOOP_RETURN", "HOLD_STILL"
    };
    private static final String[] HELP = {
            "Push straight along a tape measure. A at start, B at end. Measure the true distance.",
            "Push SIDEWAYS (left or right) along a tape measure. A at start, B at end.",
            "Spin CCW in place with the right stick, exactly N full turns back to a floor mark. A, spin, B.",
            "Spin CW in place with the right stick, exactly N full turns back to the mark. A, spin, B.",
            "Drive/push any closed loop and put the robot back on its start marks. A at start, B at end.",
            "Leave the robot untouched for 60+ s. A, wait, B. Measures drift."
    };

    private static class Snap {
        long enc[] = new long[2];
        double x, y, headingUnwrappedDeg, tMs;
    }

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        GoBildaPinpointDriver odo = robot.odo;
        robot.useFullOdometryScope();   // raw encoder ticks + status needed here

        DcMotorEx[] drive = {robot.lf, robot.lb, robot.rf, robot.rb};
        for (DcMotorEx m : drive) m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        FlightRecorder rec = FlightRecorder.start("odocal");
        java.util.Map<String, String> cfg = new java.util.LinkedHashMap<>();
        cfg.put("opmode", "OdometryCalibration");
        cfg.put("x_offset_in", String.valueOf(org.firstinspires.ftc.teamcode.PedroPathing.Constants.localizerConfig.xPodOffset.get()));
        cfg.put("y_offset_in", String.valueOf(org.firstinspires.ftc.teamcode.PedroPathing.Constants.localizerConfig.yPodOffset.get()));
        cfg.put("ticks_per_mm", String.valueOf(OdometryConfig.activeTicksPerMm()));
        cfg.put("yaw_scalar", String.valueOf(OdometryConfig.YAW_SCALAR));
        cfg.put("pivot_fwd_in", String.valueOf(OdometryConfig.TURRET_PIVOT_FORWARD_IN));
        cfg.put("pivot_left_in", String.valueOf(OdometryConfig.TURRET_PIVOT_LEFT_IN));
        rec.writeConfig(cfg);
        rec.event("--- ODOMETRY INIT ---\n" + robot.getOdometryReport());
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("ODO CALIBRATION  (Y test, A start, B end, X re-zero)");
        telemetry.addLine(rec.getStatus());
        telemetry.update();
        waitForStart();

        int test = 0, markIndex = 0;
        boolean lastA = false, lastB = false, lastX = false, lastY = false;
        Snap start = null;
        double lastHeadingDeg = odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        ElapsedTime loop = new ElapsedTime();
        ElapsedTime clock = new ElapsedTime();
        String lastResult = "";
        int lastStatus = odo.getDeviceStatus().ordinal();
        int[] statusEntries = new int[8];
        int jumps = 0;
        double maxJump = 0, prevX = odo.getPosX(DistanceUnit.INCH), prevY = odo.getPosY(DistanceUnit.INCH);
        long prevEncX = odo.getEncoderX();

        try {
            while (opModeIsActive()) {
                double dtMs = loop.milliseconds();
                loop.reset();

                long r0 = System.nanoTime();
                odo.update();
                double readUs = (System.nanoTime() - r0) / 1000.0;

                double drv = -gamepad1.left_stick_y * 0.4, str = gamepad1.left_stick_x * 0.4,
                        trn = gamepad1.right_stick_x * 0.4;
                double den = Math.max(Math.abs(drv) + Math.abs(str) + Math.abs(trn), 1.0);
                robot.lf.setPower((drv + str + trn) / den);
                robot.lb.setPower((drv - str + trn) / den);
                robot.rf.setPower((drv - str - trn) / den);
                robot.rb.setPower((drv + str - trn) / den);

                Snap now = snapshot(odo, clock.milliseconds());

                // ---- pod health: status transitions and impossible pose jumps ----
                int st = odo.getDeviceStatus().ordinal();
                if (st != lastStatus) {
                    if (st < statusEntries.length) statusEntries[st]++;
                    rec.event("POD_STATUS " + odo.getDeviceStatus() + " pose=(" + now.x + "," + now.y + ")"
                            + " encX=" + now.enc[0] + " encY=" + now.enc[1]);
                    lastStatus = st;
                }
                double jump = Math.hypot(now.x - prevX, now.y - prevY);
                if (jump > 2.0 + 80.0 * dtMs / 1000.0) {
                    jumps++;
                    maxJump = Math.max(maxJump, jump);
                    rec.event(String.format(Locale.US,
                            "POSE_JUMP %.1f in in %.0f ms status=%s dEncX=%d", jump, dtMs,
                            odo.getDeviceStatus(), now.enc[0] - prevEncX));
                }
                prevX = now.x;
                prevY = now.y;
                prevEncX = now.enc[0];

                if (gamepad1.y && !lastY) test = (test + 1) % TESTS.length;
                lastY = gamepad1.y;

                if (gamepad1.x && !lastX) {
                    for (DcMotorEx m : drive) m.setPower(0);
                    rec.event("REZERO " + robot.rezeroOdometry());
                    start = null;
                }
                lastX = gamepad1.x;

                if (gamepad1.a && !lastA) {
                    start = now;
                    markIndex++;
                    rec.event(String.format(Locale.US,
                            "MARK_START test=%s idx=%d encX=%d encY=%d x=%.4f y=%.4f hdgDeg=%.4f",
                            TESTS[test], markIndex, now.enc[0], now.enc[1], now.x, now.y,
                            now.headingUnwrappedDeg));
                }
                lastA = gamepad1.a;

                if (gamepad1.b && !lastB && start != null) {
                    lastResult = String.format(Locale.US,
                            "MARK_END test=%s idx=%d encX=%d encY=%d x=%.4f y=%.4f hdgDeg=%.4f"
                                    + " dEncX=%d dEncY=%d dx=%.4f dy=%.4f dHdgDeg=%.4f durS=%.1f",
                            TESTS[test], markIndex, now.enc[0], now.enc[1], now.x, now.y,
                            now.headingUnwrappedDeg, now.enc[0] - start.enc[0],
                            now.enc[1] - start.enc[1], now.x - start.x, now.y - start.y,
                            now.headingUnwrappedDeg - start.headingUnwrappedDeg,
                            (now.tMs - start.tMs) / 1000.0);
                    rec.event(lastResult);
                    start = null;
                }
                lastB = gamepad1.b;

                rec.put("test", test);
                rec.put("marking", start != null);
                rec.put("enc_x", now.enc[0]);
                rec.put("enc_y", now.enc[1]);
                rec.put("x_in", now.x);
                rec.put("y_in", now.y);
                rec.put("heading_unwrapped_deg", now.headingUnwrappedDeg);
                rec.put("vel_x", odo.getVelX(DistanceUnit.INCH));
                rec.put("vel_y", odo.getVelY(DistanceUnit.INCH));
                rec.put("vel_h_dps", odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                rec.put("status", odo.getDeviceStatus().ordinal());
                rec.put("pinpoint_loop_us", odo.getLoopTime());
                rec.put("pinpoint_hz", odo.getFrequency());
                rec.put("read_us", readUs);
                rec.put("dt_ms", dtMs);
                rec.put("in_drive", drv);
                rec.put("in_strafe", str);
                rec.put("in_turn", trn);
                rec.endRow();

                telemetry.addData("TEST (Y)", "%s", TESTS[test]);
                telemetry.addLine(HELP[test]);
                telemetry.addData("Pinpoint", "%s  %.0f Hz  read %.1f ms", odo.getDeviceStatus(),
                        odo.getFrequency(), readUs / 1000.0);
                telemetry.addData("Pose", "x=%.2f y=%.2f hdg(unwrapped)=%.2f", now.x, now.y,
                        now.headingUnwrappedDeg);
                telemetry.addData("POD HEALTH", "X-pod faults=%d  Y-pod faults=%d  bad reads=%d  pose jumps=%d (max %.1f in)",
                        statusEntries[3], statusEntries[4], statusEntries[7], jumps, maxJump);
                telemetry.addData("Raw ticks", "X=%d  Y=%d", now.enc[0], now.enc[1]);
                if (start != null) {
                    telemetry.addData("SINCE START", "dx=%.2f dy=%.2f dHdg=%.2f dEncX=%d dEncY=%d",
                            now.x - start.x, now.y - start.y,
                            now.headingUnwrappedDeg - start.headingUnwrappedDeg,
                            now.enc[0] - start.enc[0], now.enc[1] - start.enc[1]);
                }
                if (!lastResult.isEmpty()) telemetry.addData("Last END", lastResult);
                telemetry.addData("Log", rec.getStatus());
                telemetry.update();
            }
        } catch (Throwable t) {
            rec.exception("OdometryCalibration", t);
            throw t;
        } finally {
            for (DcMotorEx m : drive) m.setPower(0);
            rec.event("OPMODE_END");
            rec.close();
        }
    }

    private Snap snapshot(GoBildaPinpointDriver odo, double tMs) {
        Snap s = new Snap();
        s.enc[0] = odo.getEncoderX();
        s.enc[1] = odo.getEncoderY();
        s.x = odo.getPosX(DistanceUnit.INCH);
        s.y = odo.getPosY(DistanceUnit.INCH);
        s.headingUnwrappedDeg = odo.getHeading(UnnormalizedAngleUnit.DEGREES);
        s.tMs = tMs;
        return s;
    }
}
