package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Basic Auto Template", group="Linear Opmode")
public class AnnieTbot extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware variables
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Set motor directions (invert if necessary)
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to begin (Driver Press PLAY)
        waitForStart();
        runtime.reset();

        // Step 1: Drive forward for 8 seconds at 80% power
        leftDrive.setPower(0.8);
        rightDrive.setPower(0.8);
        sleep(5000);

        //Step 2: turn right
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(2000);

        // Step 3: Drive forward for 8 seconds at 80% power
        leftDrive.setPower(0.8);
        rightDrive.setPower(0.8);
        sleep(5000);

        //Step 4: turn right
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(2000);

        // Step 5: Drive forward for 8 seconds at 80% power
        leftDrive.setPower(0.8);
        rightDrive.setPower(0.8);
        sleep(5000);

        //Step 6: turn right
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
        sleep(2000);

        // Step 7: Drive forward for 8 seconds at 80% power
        leftDrive.setPower(0.8);
        rightDrive.setPower(0.8);
        sleep(5000);

        // Step : Stop the robot
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
