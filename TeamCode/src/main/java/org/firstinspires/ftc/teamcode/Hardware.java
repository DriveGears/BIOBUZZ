package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private static Hardware instance;
    public final DcMotorEx outtake;
    public final DcMotorEx intake;
    public final DcMotorEx transfer;

    public Hardware(HardwareMap hwMap) {
        this.outtake = hwMap.get(DcMotorEx.class, Specifications.OUTTAKE);
        this.intake = hwMap.get(DcMotorEx.class, Specifications.INTAKE);
        this.transfer = hwMap.get(DcMotorEx.class, Specifications.TRANSFER);

        outtake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public static Hardware getInstance(HardwareMap hwMap) {
        instance = new Hardware(hwMap);
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

}
