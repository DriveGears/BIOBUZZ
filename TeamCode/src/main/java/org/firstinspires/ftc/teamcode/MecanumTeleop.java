package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Mecanum TeleOp")
public class MecanumTeleop extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {

        Hardware robot = Hardware.getInstance(hardwareMap);

        // Match these names to your Robot Configuration
        frontLeft = robot.lf;
        frontRight = robot.rf;
        backLeft = robot.lb;
        backRight = robot.rb;

        // Reverse the right side
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Left stick: forward/backward + strafing
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            // Right stick: rotation
            double rx = gamepad1.right_stick_x;

            // Mecanum wheel calculations
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Normalize so no motor goes above 1.0
            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.abs(frontLeftPower),
                            Math.max(
                                    Math.abs(backLeftPower),
                                    Math.max(
                                            Math.abs(frontRightPower),
                                            Math.abs(backRightPower)
                                    )
                            )
                    )
            );

            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);
        }
    }
}