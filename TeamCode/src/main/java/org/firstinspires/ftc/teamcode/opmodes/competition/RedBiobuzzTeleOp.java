package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "BIOBUZZ TeleOp - RED", group = "Competition")
public class RedBiobuzzTeleOp extends biobuzzTeleop {
    @Override
    protected turretSubsystem.Alliance competitionAlliance() {
        return turretSubsystem.Alliance.RED;
    }
}
