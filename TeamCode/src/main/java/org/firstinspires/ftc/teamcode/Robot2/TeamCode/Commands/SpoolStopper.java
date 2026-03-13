package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class SpoolStopper extends SequentialCommandGroup {
    public SpoolStopper(Launcher launcher, Intake intake){
        addCommands(
                new LauncherStateCommand(launcher, Launcher.LauncherState.IDLE),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }
}
