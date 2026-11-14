package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * Bare Pinpoint connectivity test. Deliberately uses NONE of our own classes
 * (no Hardware, no Constants, no turret, no recorder) and WRITES NOTHING to the
 * device unless you press A. If this reports FAULT_BAD_READ / version 0, the
 * problem is wiring, port, power, robot configuration or the Pinpoint itself,
 * not our code.
 *
 * A (optional): run resetPosAndIMU() once (a stock driver write) and watch the status.
 */
@TeleOp(name = "PINPOINT RAW TEST", group = "TeleOp")
public class PinpointRawTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        List<GoBildaPinpointDriver> all = hardwareMap.getAll(GoBildaPinpointDriver.class);
        telemetry.addData("Pinpoint devices in robot config", all.size());
        for (GoBildaPinpointDriver d : all) {
            telemetry.addData("  name", hardwareMap.getNamesOf(d));
        }
        if (all.isEmpty()) {
            telemetry.addLine("NO goBILDA Pinpoint is configured. Fix the Robot Configuration (device 'odo').");
            telemetry.update();
            waitForStart();
            return;
        }
        GoBildaPinpointDriver odo = all.get(0);
        telemetry.addLine("Press START. A = stock resetPosAndIMU (only write this OpMode does).");
        telemetry.update();
        waitForStart();

        long loops = 0, bad = 0, ok = 0;
        boolean lastA = false;
        long t0 = System.nanoTime();
        while (opModeIsActive()) {
            long r0 = System.nanoTime();
            odo.update();
            double readMs = (System.nanoTime() - r0) / 1e6;
            GoBildaPinpointDriver.DeviceStatus st = odo.getDeviceStatus();
            loops++;
            if (st == GoBildaPinpointDriver.DeviceStatus.READY) ok++; else bad++;

            if (gamepad1.a && !lastA) odo.resetPosAndIMU();
            lastA = gamepad1.a;

            telemetry.addData("Status", st);
            telemetry.addData("Device ID / firmware", "%d / %d", odo.getDeviceID(), odo.getDeviceVersion());
            telemetry.addData("Pose", "x=%.2f y=%.2f hdg=%.1f", odo.getPosX(DistanceUnit.INCH),
                    odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Raw ticks", "X=%d Y=%d", odo.getEncoderX(), odo.getEncoderY());
            telemetry.addData("Pinpoint loop", "%.0f Hz", odo.getFrequency());
            telemetry.addData("Reads", "%d ok, %d not-ready  (%.1f ms/read, %.0f s)", ok, bad, readMs,
                    (System.nanoTime() - t0) / 1e9);
            telemetry.addLine(st == GoBildaPinpointDriver.DeviceStatus.READY
                    ? "COMMUNICATING. Move a pod wheel by hand: ticks should change."
                    : "NOT communicating cleanly: check LED, power, I2C cable/port, robot configuration.");
            telemetry.update();
            sleep(20);
        }
    }
}
