package org.firstinspires.ftc.teamcode.opmodes.competitionTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "OFFICIAL BLUE TeleOp", group = "Competition")
public class BlueTeleop extends biobuzzTeleop {
    @Override
    protected turretSubsystem.Alliance competitionAlliance() {
        return turretSubsystem.Alliance.BLUE;
    }
}
