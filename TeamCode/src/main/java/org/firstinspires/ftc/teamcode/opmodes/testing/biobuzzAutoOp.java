package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Basic Mecanum Auto", group="Auto")
public class biobuzzAutoOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        Hardware robot = Hardware.getInstance(hardwareMap);

        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Auto Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;
        //auto stuff
        drive(robot, 0.5, 100); // go forwards so that it won't crash when turning
        rotate(robot, -0.5, 167); // turn left 45
        drive(robot, 0.5, 1750); // go towards flower
        sleep(2000); // do stuff at the flower
        rotate(robot, 0.5, 394); // turn right 90
        drive(robot, 0.5, 2000); // go to other side of the field

    }
    private void drive(Hardware robot, double power, int millSecs){
        robot.lf.setPower(power);
        robot.lb.setPower(power);
        robot.rf.setPower(power);
        robot.rb.setPower(power);
        sleep(millSecs);
        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);
    }
    private void rotate(Hardware robot, double power, int millSecs){
        robot.lf.setPower(power);
        robot.lb.setPower(power);
        robot.rf.setPower(-power);
        robot.rb.setPower(-power);
        sleep(millSecs);
        robot.lf.setPower(0);
        robot.lb.setPower(0);
        robot.rf.setPower(0);
        robot.rb.setPower(0);
    }
}
