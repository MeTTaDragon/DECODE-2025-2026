package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.llta;

/**
 * Spins the robot in place using Pedro teleop drive until the Limelight detects
 * a ball cluster large enough to approach (llta > CLUSTER_TA_THRESHOLD).
 *
 * Use alongside a WaitCommand in a raceWith() to enforce a maximum scan time.
 * After this command ends, the follower is still in teleop mode — the next
 * FollowPathCommand or DynamicPathFollowCommand will switch it back.
 */
@Config
public class LimelightScanCommand extends CommandBase {

    public static double SCAN_TURN_POWER = 0.3;
    // Tune this on hardware: if robot spins the wrong direction, negate SCAN_TURN_POWER.

    public static double CLUSTER_TA_THRESHOLD = 0.5;
    // Limelight area percentage at which a cluster is considered "found".
    // Increase if getting false positives from distant single balls.

    private final Follower follower;

    public LimelightScanCommand(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
    }

    @Override
    public void execute() {
        follower.setTeleOpMovementVectors(0, 0, SCAN_TURN_POWER, false);
    }

    @Override
    public boolean isFinished() {
        return llta > CLUSTER_TA_THRESHOLD;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpMovementVectors(0, 0, 0, false);
    }
}
