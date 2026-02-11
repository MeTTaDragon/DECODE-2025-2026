package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;

public class SpoolUpCommand extends ParallelCommandGroup {
    public SpoolUpCommand(Launcher launcher, LimelightSubsystem limelight) {
        addCommands(
                new LauncherStateCommand(launcher, Launcher.LauncherState.SHOOTING),
                new StopperPoseCommand(launcher, Launcher.stopperOpen)
        );
    }
}
