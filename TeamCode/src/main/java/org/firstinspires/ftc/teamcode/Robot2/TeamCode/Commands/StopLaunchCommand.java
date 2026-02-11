package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class StopLaunchCommand extends ParallelCommandGroup {
    public StopLaunchCommand(Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight){
        addCommands(
                new LauncherStateCommand(launcher::stop),
                new StopperPoseCommand(launcher, Launcher.stopperClose),
                new TurretStateCommand(turret, Turret.TurretState.IDLE),
                new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.PAUSE),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }
}
