package org.firstinspires.ftc.teamcode.opmodes.competitionAutoOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@Autonomous(name = "Blue Auto (Upward Cell)", group = "Competition")
public class BlueAuto extends biobuzzAuto {
    @Override
    protected turretSubsystem.Alliance competitionAlliance() {
        return turretSubsystem.Alliance.BLUE;
    }
}
