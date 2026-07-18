package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;

public class SpoolUpCommand extends ParallelCommandGroup {
    public SpoolUpCommand(Launcher launcher, LimelightSubsystem limelight) {
        addCommands(
                new LauncherStateCommand(launcher, Launcher.LauncherState.SHOOTING)
                // NOTE: Limelight pipeline is no longer forced to BASKET here. Gamepad2's
                // LEFT_BUMPER/RIGHT_BUMPER are now the single source of truth for which
                // pipeline is active (BASKET vs FAR_GOAL) — spooling up shouldn't silently
                // override whichever goal the shooter driver has selected.
        );
    }
}