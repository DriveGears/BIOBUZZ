package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="Basic Mecanum TeleOp", group="TeleOp")
public class biobuzzTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hardware.resetInstance();
        Hardware robot = Hardware.getInstance(hardwareMap);

        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean intake = gamepad1.right_trigger > 0.5;
            boolean outtake = gamepad1.left_trigger > 0.5;
            boolean spit = gamepad1.right_bumper;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (intake) {
                robot.intake.setPower(0.8);
            } else if (spit) {
                robot.intake.setPower(-0.8);
            } else {
                robot.intake.setPower(0);
            }

            if (outtake) {
                robot.outtake.setPower(0.8);
                robot.transfer.setPower(0.8);
            } else {
                robot.outtake.setPower(0);
                robot.transfer.setPower(0);
            }

            robot.lf.setPower(frontLeftPower);
            robot.lb.setPower(backLeftPower);
            robot.rf.setPower(frontRightPower);
            robot.rb.setPower(backRightPower);

            telemetry.addData("Forward", y);
            telemetry.addData("Strafe", x);
            telemetry.addData("Turn", rx);

            telemetry.addData("Intake", intake);
            telemetry.addData("Outtake", outtake);
            telemetry.addData("Spit", spit);

            telemetry.addData("Front left", frontLeftPower);
            telemetry.addData("Back left", backLeftPower);
            telemetry.addData("Front right", frontRightPower);
            telemetry.addData("Back right", backRightPower);

            telemetry.update();
        }
    }
}