package org.firstinspires.ftc.teamcode.fromOldRobot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretHardware {

    public DcMotorEx panMotor = null;
    public DcMotorEx flywheelMotor = null;
    public Servo hoodServo = null;
    public Limelight3A limelight = null;

    HardwareMap hwMap = null;

    public TurretHardware() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        panMotor = hwMap.get(DcMotorEx.class, "panMotor");
        panMotor.setDirection(DcMotor.Direction.FORWARD);
        panMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        panMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        panMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        panMotor.setPower(0);

        flywheelMotor = hwMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setVelocity(0);

        hoodServo = hwMap.get(Servo.class, "hoodServo");

        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }
}