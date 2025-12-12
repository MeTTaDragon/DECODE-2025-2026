package org.firstinspires.ftc.teamcode.TeamCode.OpMode.Auto; // make sure this aligns with class location

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.TeamCode.Commands.setTunDirectionCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Blue command", group = "Auto")
public class CommandAutoBlue extends CommandOpMode {

    //TODO: scazut power tun, sa se deschida mai tarziu gateul

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private boolean actionStarted = false;
    private ElapsedTime elapsedTime = new ElapsedTime();
    Tun tun;

    private final Pose startPose = new Pose(21.5, 121.5, Math.toRadians(130)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(54, 89, Math.toRadians(128));
   private final Pose setPickupPose1 = new Pose(45, 82.5, Math.toRadians(360));
    private final Pose pickup1Pose = new Pose(15, 82.5, Math.toRadians(360));
    private  final Pose leavePose = new Pose (35, 86, Math.toRadians(   130));
    private Path scorePreload;
    private PathChain leave, grabPickup1, setPickup1, scorePickup1, grabPickup2, scorePickup2,interPickup3, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(setPickupPose1, pickup1Pose))
                .setConstantHeadingInterpolation(setPickupPose1.getHeading())
                .build();

        setPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, setPickupPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), setPickupPose1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose,scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();



        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose)).build();
    }

    private InstantCommand setTunState(Tun.tunState state) {
        return new InstantCommand(
                () -> tun.setTunState(state),
                tun // Requirement
        );
    }
    private InstantCommand setTunPower(double targetPower) {
        return new InstantCommand(
                () -> tun.setTunPower(targetPower),
                tun // Requirement
        );
    }
    private InstantCommand setBackGate(double pos){
        return new InstantCommand(() ->
                tun.setBackGatePos(pos),
                tun
        );
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
                // Score preload
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, scorePreload),
                        setBackGate(tun.gateCloseBack)

                ),

                setTunPower(0.78),
                setTunState(Tun.tunState.FORWARD),

                new WaitCommand(300),
                setBackGate(0),
                new WaitCommand(8000),

                // First pickup cycle
                new FollowPathCommand(follower, setPickup1),


                new FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.5),
                setTunState(Tun.tunState.IDLE),
                setBackGate(tun.gateCloseBack),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, scorePreload),
                        setTunState(Tun.tunState.FORWARD)
                ),
                new WaitCommand(300),
                setBackGate(0),
                new WaitCommand(9000),
                setTunState(Tun.tunState.IDLE),
                new FollowPathCommand(follower, leave),

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
