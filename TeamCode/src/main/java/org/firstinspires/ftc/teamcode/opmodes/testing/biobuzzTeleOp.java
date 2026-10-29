package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.mecanum.MecanumCommand;


@TeleOp(name="Basic Mecanum TeleOp", group="TeleOp")
public class biobuzzTeleOp extends LinearOpMode {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private double theta;
    @Override
    public void runOpMode() {
        hw               = Hardware.getInstance(hardwareMap);
        mecanumCommand   = new MecanumCommand(hw);

        Hardware.resetInstance();
        hw.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        hw.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        hw.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        hw.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            mecanumCommand.processOdometry();

            double rawHeading = mecanumCommand.getOdoHeading();
            if (Double.isNaN(rawHeading)) rawHeading = 0;

            double inputY = -applyDeadband(gamepad1.left_stick_y);
            double inputX = applyDeadband(gamepad1.left_stick_x);
            double inputR = applyDeadband(gamepad1.right_stick_x);

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
                hw.intake.setPower(0.8);
            } else if (spit) {
                hw.intake.setPower(-0.8);
            } else {
                hw.intake.setPower(0);
            }

            if (outtake) {
                hw.outtake.setPower(0.8);
                hw.transfer.setPower(0.8);
            } else {
                hw.outtake.setPower(0);
                hw.transfer.setPower(0);
            }

            hw.lf.setPower(frontLeftPower);
            hw.lb.setPower(backLeftPower);
            hw.rf.setPower(frontRightPower);
            hw.rb.setPower(backRightPower);

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

            theta = mecanumCommand.normalMove(inputY, inputX, inputR);
        }

    }

    private double applyDeadband(double value) {
        return Math.abs(value) < 0.05 ? 0 : value;
    }

    private void teleopAcknowledgeReset(Hardware robot) {
        // Reset the instance at the start of initialization if needed for safety
        Hardware.resetInstance();
        robot = Hardware.getInstance(hardwareMap);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }
}