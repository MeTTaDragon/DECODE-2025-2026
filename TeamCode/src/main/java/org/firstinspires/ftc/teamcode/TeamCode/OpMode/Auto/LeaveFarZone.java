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
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Leave blue", group = "Auto")
public class LeaveFarZone extends CommandOpMode {

    //TODO: scazut power tun, sa se deschida mai tarziu gateul

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private boolean actionStarted = false;
    private ElapsedTime elapsedTime = new ElapsedTime();
    Tun tun;
    PivotTun pivotTun;

    Pose startPose = new Pose(45, 8, Math.toRadians(90));
    Pose leave = new Pose(37, 7, Math.toRadians(90));


    private PathChain leavepath;

    public void buildPaths() {
        leavepath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leave))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }

    private InstantCommand setPivotPos(int targetPosition) {
        return new InstantCommand(
                () -> pivotTun.setPivotPosition(targetPosition), // Acțiunea (Lambda)
                pivotTun // Requirement-ul (spune scheduler-ului că folosim pivotTun)
        );
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
        pivotTun = new PivotTun(hardwareMap);
        register(tun,pivotTun);
        tun.init();
        pivotTun.init();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();


        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(

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
        telemetry.addData("pivot target", pivotTun.getTARGET_POSITION());
        telemetry.addData("pivot current", pivotTun.getCurrentPosition());
        telemetry.addData("pivot power", pivotTun.getPIVOT_POWER());

        telemetry.update();

    }

    /** This method is called continuously after Init while waiting for "play". **/

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/


}
