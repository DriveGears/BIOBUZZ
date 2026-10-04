package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
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
        // go front (go to hive in front)
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        sleep (500);

        // stop (shooting balls)
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep (10000);

        // sideways left
        frontLeft.setPower(-0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(-0.5);
        sleep(1500);

        // stop (wait at flower)
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep (5000);

        // go front
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);
        sleep (2000);

        // sideways right to hive
        frontLeft.setPower(0.5);
        frontRight.setPower(-0.5);
        backLeft.setPower(-0.5);
        backRight.setPower(0.5);
        sleep(1500);

        // stop (shooting balls)
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
}