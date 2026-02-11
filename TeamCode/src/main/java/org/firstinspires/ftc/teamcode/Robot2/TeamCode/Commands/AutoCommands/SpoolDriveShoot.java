package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolUpCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class SpoolDriveShoot extends SequentialCommandGroup {
    public SpoolDriveShoot(Follower follower, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight, boolean mixedAim){
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new SpoolUpCommand(launcher, limelight)
                ),
                new SavePoseCommand(follower),
                new ShootCommand(launcher, limelight, turret, intake, mixedAim),
                new WaitCommand(800),
                new StopLaunchCommand(launcher, turret, intake, limelight)
        );
    }
}
