package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.brakingpower;

/**
 * Builds and follows a straight-line path from the robot's current pose to a
 * fixed target pose. The path is constructed lazily in initialize() so the
 * start position is always the robot's actual pose at the moment the command runs.
 *
 * Use this for post-vision navigation where the robot's position after intake
 * is not known ahead of time.
 */
public class DynamicPathFollowCommand extends CommandBase {

    private final Follower follower;
    private final Pose targetPose;
    private final double targetHeadingRad;

    /**
     * @param follower         Pedro Pathing follower
     * @param targetPose       Destination pose (x, y ignored for heading; use targetHeadingRad)
     * @param targetHeadingRad Final heading in radians at the target pose
     */
    public DynamicPathFollowCommand(Follower follower, Pose targetPose, double targetHeadingRad) {
        this.follower = follower;
        this.targetPose = targetPose;
        this.targetHeadingRad = targetHeadingRad;
    }

    @Override
    public void initialize() {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetPose))
                .setLinearHeadingInterpolation(follower.getHeading(), targetHeadingRad)
                .setBrakingStrength(Constants.pathConstraints.getBrakingStrength())
                .setBrakingStart(Constants.pathConstraints.getBrakingStart())
                .setGlobalDeceleration(brakingpower)
                .build();
        follower.followPath(path, true);
    }

    @Override
    public void execute() {
        // follower.update() is called in the opmode's run() each loop cycle
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
