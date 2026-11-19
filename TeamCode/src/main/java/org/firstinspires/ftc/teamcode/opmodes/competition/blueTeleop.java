package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.turretSubsystem;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class blueTeleop extends biobuzzTeleop {
    @Override
    protected turretSubsystem.Alliance competitionAlliance() {
        return turretSubsystem.Alliance.BLUE;
    }
}
