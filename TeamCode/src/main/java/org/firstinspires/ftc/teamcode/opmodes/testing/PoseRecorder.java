package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.tools.PoseStore;

import java.util.Map;

/**
 * Push the robot by hand (drive motors are unpowered + floating) to each spot
 * and save it. Poses are written to the robot's storage immediately.
 *
 * Place the robot at the auto START pose before pressing START so the
 * coordinates match redAuto's field frame.
 *
 * Gamepad 1:
 *   DPAD UP / DOWN : choose which label to record
 *   A              : save current pose under the selected label (overwrites), then advance
 *   B              : delete selected label
 *   X              : re-zero pose to the start pose (robot placed at start)
 *   Y + LEFT BUMPER: clear all saved poses
 */
@TeleOp(name = "Pose Recorder", group = "Testing")
public class PoseRecorder extends OpMode {
    // Same labels / names used in redAuto. Add more here as needed.
    private static final String[] LABELS = {
            "flowerPose", "flower2LineupPose", "flowerPose2"
    };
    private static final Pose START = new Pose(32.0, 9.1, Math.toRadians(90));

    private Follower follower;
    private Hardware robot;
    private Map<String, PoseStore.P> poses;
    private int sel = 0;
    private String status = "Ready";
    private boolean pUp, pDown, pA, pB, pX;

    @Override
    public void init() {
        Hardware.resetInstance();
        robot = Hardware.getInstance(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(START);
        follower.update();
        poses = PoseStore.load();
        PoseStore.beginSession();
        floatDrive();
        status = "Loaded " + poses.size() + " saved pose(s)";
    }

    @Override
    public void init_loop() {
        follower.update();
        floatDrive();
        telemetry.addLine("Place robot at START pose, then press PLAY (or just push it).");
        show();
    }

    @Override
    public void loop() {
        follower.update();
        floatDrive();

        if (gamepad1.dpad_up && !pUp) sel = (sel + LABELS.length - 1) % LABELS.length;
        if (gamepad1.dpad_down && !pDown) sel = (sel + 1) % LABELS.length;

        if (gamepad1.a && !pA) {
            Pose p = follower.pose();
            PoseStore.P rec = new PoseStore.P(p.x(), p.y(), Math.toDegrees(p.heading()));
            poses.put(LABELS[sel], rec);
            String err = PoseStore.save(poses);
            String logErr = PoseStore.appendOne(LABELS[sel], rec);
            if (err == null && logErr != null) err = "history: " + logErr;
            status = err == null ? "Saved " + LABELS[sel] : "SAVE FAILED: " + err;
            sel = (sel + 1) % LABELS.length;
        }
        if (gamepad1.b && !pB) {
            poses.remove(LABELS[sel]);
            String err = PoseStore.save(poses);
            status = err == null ? "Deleted " + LABELS[sel] : "SAVE FAILED: " + err;
        }
        if (gamepad1.x && !pX) {
            follower.setPose(START);
            status = "Pose re-zeroed to start";
        }
        if (gamepad1.y && gamepad1.left_bumper) {
            poses.clear();
            String err = PoseStore.save(poses);
            status = err == null ? "Cleared all" : "SAVE FAILED: " + err;
        }
        pUp = gamepad1.dpad_up; pDown = gamepad1.dpad_down;
        pA = gamepad1.a; pB = gamepad1.b; pX = gamepad1.x;
        show();
    }

    private void show() {
        Pose p = follower.pose();
        telemetry.addData("Status", status);
        telemetry.addData("Live pose", "x=%.1f y=%.1f h=%.0f°", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addLine("DPAD U/D select | A save | B delete | X rezero | LB+Y clear all");
        for (int i = 0; i < LABELS.length; i++) {
            PoseStore.P s = poses.get(LABELS[i]);
            telemetry.addData((i == sel ? "> " : "  ") + LABELS[i],
                    s == null ? "(not saved)" : String.format("%.1f, %.1f, %.0f°", s.x, s.y, s.headingDeg));
        }
        telemetry.addData("File", PoseStore.CSV.getPath());
        telemetry.update();
    }

    /** Keep wheels free-spinning so the robot can be pushed. */
    private void floatDrive() {
        for (DcMotorEx m : new DcMotorEx[]{robot.lf, robot.lb, robot.rf, robot.rb}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setPower(0);
        }
    }

    @Override
    public void stop() {
        if (robot != null) {
            for (DcMotorEx m : new DcMotorEx[]{robot.lf, robot.lb, robot.rf, robot.rb}) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}
