package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.MixedShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolUpCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

/**
 * A command that spools up the launcher while driving along a path, then stops and shoots.
 */
public class SpoolDriveShoot extends SequentialCommandGroup {
    /**
     * Constructs a SpoolDriveShoot command.
     *
     * @param follower The follower to use for path following.
     * @param path The path to follow.
     * @param launcher The launcher subsystem.
     * @param turret The turret subsystem.
     * @param intake The intake subsystem.
     * @param limelight The limelight subsystem.
     * @param mixedAim Whether to use mixed aiming.
     */
    public SpoolDriveShoot(Follower follower, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight){
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new SpoolUpCommand(launcher, limelight)
                ),
                new SavePoseCommand(follower),
                new MixedShootCommand(launcher, turret, intake, limelight),
                new WaitCommand(800),
                new StopLaunchCommand(launcher, turret, intake, limelight)
        );
    }

    /**
     * Constructs a SpoolDriveShoot command with a specified maximum speed.
     *
     * @param follower The follower to use for path following.
     * @param maxSpeed The maximum speed to follow the path at.
     * @param path The path to follow.
     * @param launcher The launcher subsystem.
     * @param turret The turret subsystem.
     * @param intake The intake subsystem.
     * @param limelight The limelight subsystem.
     * @param mixedAim Whether to use mixed aiming.
     */
    public SpoolDriveShoot(Follower follower, double maxSpeed, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight){
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new SpoolUpCommand(launcher, limelight)
                ),
                new SavePoseCommand(follower),
                new MixedShootCommand(launcher, turret, intake, limelight),
                new WaitCommand(800),
                new StopLaunchCommand(launcher, turret, intake, limelight)
        );
    }
}
