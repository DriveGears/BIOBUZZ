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
        Hardware robot = Hardware.getInstance(hardwareMap);

        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        teleopAcknowledgeReset(robot);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.lf.setPower(frontLeftPower);
            robot.lb.setPower(backLeftPower);
            robot.rf.setPower(frontRightPower);
            robot.rb.setPower(backRightPower);

            telemetry.addData("Forward", y);
            telemetry.addData("Strafe", x);
            telemetry.addData("Turn", rx);

            telemetry.addData("Front left", frontLeftPower);
            telemetry.addData("Back left", backLeftPower);
            telemetry.addData("Front right", frontRightPower);
            telemetry.addData("Back right", backRightPower);

            telemetry.update();
        }
    }

    private void teleopAcknowledgeReset(Hardware robot) {
        // Reset the instance at the start of initialization if needed for safety
        Hardware.resetInstance();
        robot = Hardware.getInstance(hardwareMap);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }
}
