package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.IntakeDrive;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.SpoolDriveShoot;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;


public class BlueClose12ball extends CommandOpMode {
    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.65; // Ensure this is defined
    private Follower follower;

    private final Pose startPose = new Pose(27.000, 126.534, Math.toRadians(180));
    private PathChain path1, path2, path3, path4,path4_1, path5, path6, path7, path8;

    public void buildPaths() {
        // Path 1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(27.000, 126.534), new Pose(55, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55, 84.000), new Pose(17.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(17.000, 84.000), new Pose(55, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(55, 84.000), new Pose(73.000, 55.000), new Pose(23.000, 58.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        path4_1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(73, 55.000), new Pose(17, 69)))
                .setConstantHeadingInterpolation(Math.toRadians(225))
                .build();

        // Path 5
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(17, 69), new Pose(55, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 6
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(55, 84.000), new Pose(72.000, 32.000), new Pose(23.000, 35.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 7
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(23.000, 35.000), new Pose(55, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 8
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55, 84.000), new Pose(32.854, 82.732)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }


    @Override
    public void initialize() {
        super.reset();
        alliance = Alliance.BLUE;

        // Initialize Follower and Subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower);

        register(turret, launcher, intake, limelight);

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new SpoolDriveShoot(follower, path1, launcher, turret, intake, limelight, true),

                new IntakeDrive(follower, path2, intake, 500),

                new SpoolDriveShoot(follower, path3, launcher, turret, intake, limelight, true),

                new IntakeDrive(follower, path4, intake, 500),

                new FollowPathCommand(follower, path4_1),
                new SavePoseCommand(follower),
                new WaitCommand(325),

                new SpoolDriveShoot(follower, path5, launcher, turret, intake, limelight, true),

                new IntakeDrive(follower, path6, intake, 500),

                new SpoolDriveShoot(follower, path7, launcher, turret, intake, limelight, true),

                new FollowPathCommand(follower, path8),
                new SavePoseCommand(follower)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void initialize_loop(){
        telemetry.addLine(
                "OpMode selected"
        );
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}