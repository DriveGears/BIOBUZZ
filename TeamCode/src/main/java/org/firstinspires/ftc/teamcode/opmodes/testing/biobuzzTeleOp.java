package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.shooterSubsystem;
import org.firstinspires.ftc.teamcode.turretSubsystem2;

@TeleOp(name = "BioBuzz TeleOp", group = "TeleOp")
public class biobuzzTeleOp extends LinearOpMode {

    private static final double DRIVE_SPEED = 0.80;
    private static final double STRAFE_MULTIPLIER = 1.10;
    private static final double MANUAL_TURRET_POWER = 0.25;

    private boolean lastShooterButton = false;

    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);
        turretSubsystem2 turret = new turretSubsystem2(robot);
        shooterSubsystem shooter = new shooterSubsystem(robot);
        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("BioBuzz TeleOp Ready");
        telemetry.addLine("G1 Left Stick: Drive | Right Trigger: Hold Auto-Aim");
        telemetry.addLine("G1 D-pad L/R: Manual Turret | A: Toggle Shooter/Intakes | B: Stop");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            turret.stop();
            shooter.stop();
            return;
        }

        loopTimer.reset();
        try {
            while (opModeIsActive()) {
                double dtSeconds = loopTimer.seconds();
                loopTimer.reset();

                // 1. Mecanum Drive
                double forward = -applyDeadband(gamepad1.left_stick_y) * DRIVE_SPEED;
                double strafe = applyDeadband(gamepad1.left_stick_x) * STRAFE_MULTIPLIER * DRIVE_SPEED;
                double turn = applyDeadband(gamepad1.right_stick_x) * DRIVE_SPEED;

                double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
                robot.lf.setPower((forward + strafe + turn) / denominator);
                robot.lb.setPower((forward - strafe + turn) / denominator);
                robot.rf.setPower((forward - strafe - turn) / denominator);
                robot.rb.setPower((forward + strafe - turn) / denominator);

                // 2. Shooter / Outtake Controls (A to toggle, B to stop)
                boolean aPressed = gamepad1.a;
                if (gamepad1.b) {
                    shooter.setEnabled(false);
                } else if (aPressed && !lastShooterButton) {
                    shooter.setEnabled(!shooter.isEnabled());
                }
                lastShooterButton = aPressed;

                // Set target RPM for distance if turret has distance, otherwise default
                if (turret.hasDistance()) {
                    shooter.setTargetForDistance(turret.getImpactDistanceIn());
                } else {
                    shooter.setTargetRpm(2175); // default corner shot RPM
                }
                shooter.update(dtSeconds);

                // 3. Turret Auto-Aim & Manual Control
                boolean autoAim = gamepad1.right_trigger > 0.20;
                turret.setAutoAimEnabled(autoAim);

                double manualTurret = (gamepad1.dpad_right ? 1.0 : 0.0) - (gamepad1.dpad_left ? 1.0 : 0.0);
                turret.setManualPower(manualTurret * MANUAL_TURRET_POWER);
                turret.update(dtSeconds);

                // 4. Telemetry
                telemetry.addData("Drive", "Fwd=%.2f, Strafe=%.2f, Turn=%.2f", forward, strafe, turn);
                telemetry.addData("Turret Mode", autoAim ? "AUTO-AIM" : "MANUAL");
                telemetry.addData("Target Visible", turret.hasTarget() ? "YES (ID: " + turret.getSelectedTagId() + ")" : "NO");
                telemetry.addData("Aim Ready?", turret.isAimReady() ? "READY" : "ALIGNING");
                telemetry.addData("Shooter", "Enabled=%s, Actual RPM=%.0f", shooter.isEnabled() ? "ON" : "OFF", shooter.getActualRpm());
                telemetry.update();
            }
        } finally {
            robot.lf.setPower(0);
            robot.lb.setPower(0);
            robot.rf.setPower(0);
            robot.rb.setPower(0);
            turret.stop();
            shooter.stop();
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }
}
