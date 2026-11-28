package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@Autonomous(name = "Red Auto (Upward Cell)", group = "Competition")
public class RedAuto extends biobuzzAuto {
    @Override
    protected turretSubsystem.Alliance competitionAlliance() {
        return turretSubsystem.Alliance.RED;
    }
}
