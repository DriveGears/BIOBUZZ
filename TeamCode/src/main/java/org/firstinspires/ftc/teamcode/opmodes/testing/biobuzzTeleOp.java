package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.mecanum.MecanumCommand;


@TeleOp(name="Basic Mecanum TeleOp", group="TeleOp")
public class biobuzzTeleOp extends LinearOpMode {
    private Hardware hw;
    private MecanumCommand mecanumCommand;
    private double theta;
    @Override
    public void runOpMode() {
        hw               = Hardware.getInstance(hardwareMap);
        mecanumCommand   = new MecanumCommand(hw);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            mecanumCommand.processOdometry();

            double rawHeading = mecanumCommand.getOdoHeading();
            if (Double.isNaN(rawHeading)) rawHeading = 0;

            double inputY = gamepad1.left_stick_y;
            double inputX =  gamepad1.left_stick_x;
            double inputR =  gamepad1.right_stick_x;

            theta = mecanumCommand.normalMove(inputY, inputX, inputR);
        }

    }

    private void teleopAcknowledgeReset(Hardware robot) {
        // Reset the instance at the start of initialization if needed for safety
        Hardware.resetInstance();
        robot = Hardware.getInstance(hardwareMap);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }
}
