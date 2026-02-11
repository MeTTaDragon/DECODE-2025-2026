package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;

public class IntakeDrive extends SequentialCommandGroup {
    public IntakeDrive(Follower follower, PathChain path, Intake intake, long waitTime) {
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new IntakeStateCommand(intake, Intake.IntakeState.REVERSE)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(waitTime),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );
    }
}
