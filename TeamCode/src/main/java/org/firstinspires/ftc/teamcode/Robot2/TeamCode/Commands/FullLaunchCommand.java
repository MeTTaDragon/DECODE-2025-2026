package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class FullLaunchCommand extends SequentialCommandGroup {
    public FullLaunchCommand(Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight) {
        addCommands(
                new ParallelCommandGroup(
                        new LauncherStateCommand(launcher, Launcher.LauncherState.SHOOTING),
                        new TurretStateCommand(turret, Turret.TurretState.FULL_PINPOINT),
                        new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET)
                ),

                new WaitUntilCommand(launcher::isVelocityReached),

                new TurretStateCommand(turret, Turret.TurretState.FULL_LIMELIGHT),
                new StopperPoseCommand(launcher, Launcher.stopperOpen),
                new WaitCommand(500),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE)
        );
    }

}
