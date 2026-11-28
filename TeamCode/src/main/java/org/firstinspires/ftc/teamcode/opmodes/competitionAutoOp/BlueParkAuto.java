package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

/** Park-only autonomous using the shared path with blue AprilTag selection. */
@Autonomous(name = "Blue Auto (Park Only)", group = "Competition")
public class BlueParkAuto extends biobuzzAuto {
    private static final int PARK_ONLY_TIMEOUT_MS = 8000;

    @Override
    protected turretSubsystem.Alliance competitionAlliance() {
        // The turret will consider only the upward blue CELL: official tag
        // clusters 38-41 (audience) and 42-45 (scoring).
        return turretSubsystem.Alliance.BLUE;
    }

    @Override
    protected Pose getParkPose() {
        // Same physical path as RedAuto (Park Only); only the target alliance changes.
        return poseFactory.of(1.3, 118.6, 2.0);
    }

    private Path startToPark() {
        return line(getStartPose(), getParkPose())
                .linear(getStartPose(), getParkPose());
    }

    @Override
    protected Command autoRoutine() {
        return sequential(
                instant(() -> {
                    shooter.setShooterEnabled(false);
                    robot.intake.setPower(0);
                    robot.transfer.setPower(0);
                    turret.setAutoAimEnabled(false);
                }),
                drive(startToPark(), PARK_ONLY_TIMEOUT_MS)
        );
    }
}
