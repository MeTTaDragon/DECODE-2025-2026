package org.firstinspires.ftc.teamcode.Robot1.TeamCode.OpMode.Auto; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.Robot1.pedroPathing.Constants;

@Autonomous(name = "Leave red far", group = "Auto")
public class LeaveFarZoneRed extends CommandOpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private boolean actionStarted = false;
    private ElapsedTime elapsedTime = new ElapsedTime();
    Tun tun;

    Pose startPose = new Pose(96, 8, Math.toRadians(90));
    Pose leave = new Pose(109, 8, Math.toRadians(90));


    private PathChain leavepath;

    public void buildPaths() {
        leavepath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leave))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }




    /** This method is called once at the init of the OpMode. **/
    @Override
    public void initialize() {
        super.reset();
        tun = new Tun(hardwareMap);
        register(tun);
        tun.init();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
            new FollowPathCommand(follower, leavepath),
            new InstantCommand(() -> Globals.lastAutoPose = follower.getPose())

        );
        schedule(autonomousSequence);

    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // These loop the movements of the robot, these must be called continuously in order to work

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.update();

    }

    /** This method is called continuously after Init while waiting for "play". **/

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/


}
