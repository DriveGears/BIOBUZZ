package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private static Hardware instance;
    public final DcMotorEx lf;
    public final DcMotorEx rf;
    public final DcMotorEx lb;
    public final DcMotorEx rb;

    private Hardware(HardwareMap hwMap) {
        this.rf = hwMap.get(DcMotorEx.class, Specifications.FTRT_MOTOR); //rightforward
        this.lf = hwMap.get(DcMotorEx.class, Specifications.FTLF_MOTOR); //leftforward
        this.lb = hwMap.get(DcMotorEx.class, Specifications.BKLF_MOTOR); //leftback
        this.rb = hwMap.get(DcMotorEx.class, Specifications.BKRT_MOTOR); //rightback
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
