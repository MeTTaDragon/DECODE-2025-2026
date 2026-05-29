package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.llta;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.lltx;

/**
 * Drives the robot toward a detected ball cluster using Limelight tx/ta feedback.
 *
 * The heading correction is proportional to lltx (horizontal offset from camera center).
 * When lltx == 0, the intake is pointed directly at the cluster.
 *
 * This command ends only when the cluster is lost (llta drops below LOST_TA_THRESHOLD
 * for more than 0.5 seconds). The actual "robot is full" stop comes from CheckLoadCommand
 * running in the parent ParallelRaceGroup alongside this command.
 *
 * Tune KP_HEADING sign: if the robot steers away from the cluster, flip the sign.
 */
@Config
public class LimelightBallFollowCommand extends CommandBase {

    public static double FORWARD_POWER = 0.35;
    public static double KP_HEADING = 0.03;
    // Positive tx = cluster is to the right. KP_HEADING * lltx drives the turn correction.
    // If the robot steers away from the cluster, negate KP_HEADING.

    public static double LOST_TA_THRESHOLD = 0.1;
    // Area below this for more than LOST_TIMEOUT_S means the cluster is gone.

    private static final double LOST_TIMEOUT_S = 0.5;

    private final Follower follower;
    private final ElapsedTime lostTimer = new ElapsedTime();
    private boolean wasDetecting = false;

    public LimelightBallFollowCommand(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
        lostTimer.reset();
        wasDetecting = llta > LOST_TA_THRESHOLD;
    }

    @Override
    public void execute() {
        double turnCorrection = KP_HEADING * lltx;
        follower.setTeleOpMovementVectors(FORWARD_POWER, 0, turnCorrection, false);
    }

    @Override
    public boolean isFinished() {
        boolean detecting = llta > LOST_TA_THRESHOLD;
        if (detecting) {
            wasDetecting = true;
            lostTimer.reset();
        }
        // Only time out from lost if we actually saw the cluster at least once
        return wasDetecting && lostTimer.seconds() > LOST_TIMEOUT_S;
    }

    @Override
    public void end(boolean interrupted) {
        follower.setTeleOpMovementVectors(0, 0, 0, false);
    }
}
