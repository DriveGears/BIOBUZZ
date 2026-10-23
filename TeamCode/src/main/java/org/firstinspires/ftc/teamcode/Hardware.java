package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private static Hardware instance;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;
    public final DcMotorEx turret;
    public final DcMotorEx flywheel;
    public final DcMotorEx intake;
    public final DcMotorEx transfer;
    public final Limelight3A limelight;
    public final GoBildaPinpointDriver odo;

    public Hardware(HardwareMap hwMap) {
        lf = configureDriveMotor(hwMap, Specifications.LF_MOTOR, DcMotor.Direction.REVERSE);
        lb = configureDriveMotor(hwMap, Specifications.LB_MOTOR, DcMotor.Direction.REVERSE);
        rf = configureDriveMotor(hwMap, Specifications.RF_MOTOR, DcMotor.Direction.FORWARD);
        rb = configureDriveMotor(hwMap, Specifications.RB_MOTOR, DcMotor.Direction.FORWARD);

        turret = hwMap.get(DcMotorEx.class, Specifications.TURRET);
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0);

        flywheel = hwMap.get(DcMotorEx.class, Specifications.OUTTAKE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocity(0);

        intake = configureMechanismMotor(hwMap, Specifications.INTAKE);
        transfer = configureMechanismMotor(hwMap, Specifications.TRANSFER);

        limelight = hwMap.get(Limelight3A.class, Specifications.LIMELIGHT);
        odo = hwMap.get(GoBildaPinpointDriver.class, Specifications.ODO);
    }

    private DcMotorEx configureDriveMotor(HardwareMap hwMap, String name, DcMotor.Direction direction) {
        DcMotorEx motor = hwMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        return motor;
    }

    private DcMotorEx configureMechanismMotor(HardwareMap hwMap, String name) {
        DcMotorEx motor = hwMap.get(DcMotorEx.class, name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        return motor;
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        if (instance == null) {
            instance = new Hardware(hwMap);
        }
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

}
