package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.LauncherStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.LimelightModeCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopperPoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class AutoStopLaunchCommand extends SequentialCommandGroup {
    public AutoStopLaunchCommand(Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight){
        addCommands(
                new LauncherStateCommand(launcher, Launcher.LauncherState.IDLE),
                new StopperPoseCommand(launcher, Launcher.stopperOpen),
                new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.PAUSE),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }
}
