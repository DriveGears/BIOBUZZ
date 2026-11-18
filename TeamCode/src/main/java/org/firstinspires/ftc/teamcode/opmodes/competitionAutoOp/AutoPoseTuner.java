package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@TeleOp(name = "Auto Pose Tuner TeleOp", group = "Tuning")
public class AutoPoseTuner extends OpMode {
    private Follower follower;
    private Hardware robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Streamlined target poses matching your updated auto routine
    private final String[] poseNames = {
            "startPose",
            "WflowerPose",
            "NflowerPose",
            "parkPose"
    };

    private final Pose[] loggedPoses = new Pose[poseNames.length];
    private int selectedIndex = 0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastButtonA = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        robot = Hardware.getInstance(hardwareMap);

        for (int i = 0; i < loggedPoses.length; i++) {
            loggedPoses[i] = poseFactory.of(0, 0, 0);
        }

        follower.update();
        telemetry.addData("Status", "Tuner Ready. Use D-Pad to select slot, [A] to save.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        // --- CONTROLS ---
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;

        if (dpadUp && !lastDpadUp) {
            selectedIndex = (selectedIndex + 1) % poseNames.length;
        }
        if (dpadDown && !lastDpadDown) {
            selectedIndex = (selectedIndex - 1 + poseNames.length) % poseNames.length;
        }
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;

        // Button A locks the current odometry position into the selected slot
        boolean buttonA = gamepad1.a;
        if (buttonA && !lastButtonA) {
            double x = Math.round(follower.pose().x() * 100.0) / 100.0;
            double y = Math.round(follower.pose().y() * 100.0) / 100.0;
            double heading = Math.round(Math.toDegrees(follower.pose().heading()) * 1000.0) / 1000.0;

            loggedPoses[selectedIndex] = poseFactory.of(x, y, heading);

            // Print clean Java declaration line to Driver Station log history
            telemetry.log().add(String.format("private final Pose %s = poseFactory.of(%.2f, %.2f, %.3f);",
                    poseNames[selectedIndex], x, y, heading));
        }
        lastButtonA = buttonA;

        // --- TELEMETRY UI ---
        telemetry.addData("=== INSTRUCTIONS ===", "");
        telemetry.addData("D-Pad Up/Down", "Select Pose Slot");
        telemetry.addData("Button A", "Save Odometry to Selected Slot");
        telemetry.addData("--------------------", "");
        telemetry.addData("-> EDITING SLOT", "[ %s ]", poseNames[selectedIndex]);
        telemetry.addData("Live Odometry", "X: %.2f | Y: %.2f | H: %.1f°",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("--------------------", "SAVED POSES:");

        for (int i = 0; i < poseNames.length; i++) {
            Pose p = loggedPoses[i];
            String marker = (i == selectedIndex) ? " >>> " : "     ";
            telemetry.addData(marker + poseNames[i], "X: %.2f, Y: %.2f, H: %.1f°",
                    p.x(), p.y(), Math.toDegrees(p.heading()));
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
    }
}