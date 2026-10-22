package org.firstinspires.ftc.teamcode.fromOldRobot;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    // Declare the 4 motors
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;

    // Limelight 3A camera, plugged into the USB 3.0 port on the Control Hub
    public Limelight3A limelight = null;

    // ---- Rev 2m distance sensors (VL53L0X), one per side ----
    // Mount each facing straight out from the center of its side.
    // The VL53L0X has a FIXED I2C address (0x29), so no two of these can share
    // a bus -- give each its own I2C bus on the hub(s).
    public DistanceSensor distFront = null;
    public DistanceSensor distBack = null;
    public DistanceSensor distLeft = null;
    public DistanceSensor distRight = null;

    HardwareMap hwMap = null;

    public Hardware() {}

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Initialize motors using the names configured on the Driver Station
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");

        // Set motor directions (reverse left side if standard mecanum/tank layout)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Zero power behavior (BRAKE prevents coasting)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // ---- Distance sensor setup ----
        // Names must match the Driver Station config exactly (case-sensitive).
        // Configure each as "REV 2m Distance Sensor".
        distFront = hwMap.get(DistanceSensor.class, "distFront");
        distBack  = hwMap.get(DistanceSensor.class, "distBack");
        distLeft  = hwMap.get(DistanceSensor.class, "distLeft");
        distRight = hwMap.get(DistanceSensor.class, "distRight");

        // ---- Limelight setup ----
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }
}