package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.MixedShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.ShootOnFlyCommand;
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
     * @param waitBeforeStop The time to wait in milliseconds after shooting before stopping the launcher and turret.
     */
    public SpoolDriveShoot(Follower follower, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight, long waitBeforeStop){
        addCommands(
                new FollowPathCommand(follower, path).alongWith(
                        new SpoolUpCommand(launcher, limelight)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(200),
                new MixedShootCommand(launcher, turret, intake),
                new WaitCommand(waitBeforeStop),
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
     * @param waitBeforeStop The time to wait in milliseconds after shooting before stopping the launcher and turret.
     */
    public SpoolDriveShoot(Follower follower, double maxSpeed, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight, long waitBeforeStop){
        addCommands(
                new FollowPathCommand(follower, path, maxSpeed).alongWith(
                        new SpoolUpCommand(launcher, limelight)
                ),
                new SavePoseCommand(follower),
                new WaitCommand(200),
                new MixedShootCommand(launcher, turret, intake),
                new WaitCommand(waitBeforeStop),
                new StopLaunchCommand(launcher, turret, intake, limelight)
        );
    }/**
     * Constructs a SpoolDriveShoot command with SOF.
     *
     * @param follower The follower to use for path following.
     * @param path The path to follow.
     * @param launcher The launcher subsystem.
     * @param turret The turret subsystem.
     * @param intake The intake subsystem.
     * @param limelight The limelight subsystem.
     * @param waitBeforeStop The time to wait in milliseconds after shooting before stopping the launcher and turret.
     * @param SOF shoot on the fly mode - ONLY FOR AUTO START
     */
    public SpoolDriveShoot(Follower follower, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight, long waitBeforeStop, boolean SOF){
        if(SOF){
            addCommands(
                    new FollowPathCommand(follower, path).alongWith(
                            new SpoolUpCommand(launcher, limelight)
                    ),
                    new SavePoseCommand(follower),
                    new WaitCommand(200),
                    new ShootOnFlyCommand(launcher, turret, intake),
                    new WaitCommand(waitBeforeStop),
                    new StopLaunchCommand(launcher, turret, intake, limelight)
            );
        }
        else{
            new SpoolDriveShoot(follower, path, launcher, turret, intake, limelight, waitBeforeStop);
        }

    }

    /**
     * Constructs a SpoolDriveShoot command with a specified maximum speed with SOF.
     *
     * @param follower The follower to use for path following.
     * @param maxSpeed The maximum speed to follow the path at.
     * @param path The path to follow.
     * @param launcher The launcher subsystem.
     * @param turret The turret subsystem.
     * @param intake The intake subsystem.
     * @param limelight The limelight subsystem.
     * @param waitBeforeStop The time to wait in milliseconds after shooting before stopping the launcher and turret.
     * @param SOF  shoot on the fly mode - ONLY FOR AUTO START
     */
    public SpoolDriveShoot(Follower follower, double maxSpeed, PathChain path, Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight, long waitBeforeStop, boolean SOF){
        if(SOF){
            addCommands(
                    new FollowPathCommand(follower, path, maxSpeed).alongWith(
                            new SpoolUpCommand(launcher, limelight)
                    ),
                    new SavePoseCommand(follower),
                    new WaitCommand(200),
                    new ShootOnFlyCommand(launcher, turret, intake),
                    new WaitCommand(waitBeforeStop),
                    new StopLaunchCommand(launcher, turret, intake, limelight)
            );
        }
        else{
            new SpoolDriveShoot(follower, maxSpeed, path, launcher, turret, intake, limelight, waitBeforeStop);
        }
    }
}
