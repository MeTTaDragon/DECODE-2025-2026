package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;

public class LimelightModeCommand extends InstantCommand {
    LimelightSubsystem.LimelightMode mode;
    LimelightSubsystem limelight;

    public LimelightModeCommand(LimelightSubsystem limelightSubsystem, LimelightSubsystem.LimelightMode mode) {
        this.limelight = limelightSubsystem;
        this.mode = mode;
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelight.setMode(mode);
    }
}
