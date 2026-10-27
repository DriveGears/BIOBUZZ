package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private static Hardware instance;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;
    public final DcMotorEx outtake;
    public final DcMotorEx intake;
    public final DcMotorEx transfer;
    public final DcMotorEx turret;
    public Limelight3A limelight = null;

    public Hardware(HardwareMap hwMap) {
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR);
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR);
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR);
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR);
        this.outtake = hwMap.get(DcMotorEx.class, Specifications.OUTTAKE);
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.transfer = hwMap.get(DcMotorEx.class, Specifications.TRANSFER);
        this.turret = hwMap.get(DcMotorEx.class, Specifications.TURRET);
        this.limelight = hwMap.get(Limelight3A.class,Specifications.LIMELIGHT);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        outtake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setDirection(DcMotorEx.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        instance = new Hardware(hwMap);
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

}
