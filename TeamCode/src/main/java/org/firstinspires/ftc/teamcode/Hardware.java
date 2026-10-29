package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private static Hardware instance;

    public final DcMotorEx lf;
    public static double fieldHeadingOffset = 0.0;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;

    public final DcMotorEx outtake;
    public final DcMotorEx intake;
    public final DcMotorEx transfer;

    public final GoBildaPinpointDriver pinPointOdo;

    public Hardware(HardwareMap hwMap) {
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback

        this.pinPointOdo = hwMap.get(GoBildaPinpointDriver.class, "odo");

        this.outtake = hwMap.get(DcMotorEx.class, Specifications.OUTTAKE);
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.transfer = hwMap.get(DcMotorEx.class, Specifications.TRANSFER);

        outtake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
