package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.AutoFRI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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

@Autonomous(group = "redclose", name = "red close no gate test")
public class RedCloseNoGateTest extends CommandOpMode {

    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.65;
    private Follower follower;

    // Define Poses cleanly at the top level
    private final Pose startPose = new Pose(117, 126.534, Math.toRadians(0));
    private final Pose shooting_pose = new Pose(88.000, 84.000, Math.toRadians(0));
    private final Pose spike1 = new Pose(123.000, 84.000, Math.toRadians(0));
    private final Pose spike2 = new Pose(121.000, 58.000, Math.toRadians(0));
    private final Pose spike3 = new Pose(121.000, 35.000, Math.toRadians(0));
    private final Pose exit_zone = new Pose(112.854, 82.732, Math.toRadians(0));

    // Control Points for Curves
    private final Pose ctrl_spike2 = new Pose(71.000, 55.000);
    private final Pose ctrl_spike3 = new Pose(72.000, 32.000);

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    public void buildPaths() {
        // Path 1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shooting_pose, spike1))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(spike1, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose, ctrl_spike2, spike2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 5
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(spike2, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 6
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose, ctrl_spike3, spike3))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 7
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(spike3, shooting_pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 8
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(shooting_pose, exit_zone))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.RED;

        // Initialize Follower via your updated Constants
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower);

        register(turret, launcher, intake, limelight);

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new SpoolDriveShoot(follower, path1, launcher, turret, intake, limelight, 500),
                new IntakeDrive(follower, path2, intake, 500),
                new SpoolDriveShoot(follower, path3, launcher, turret, intake, limelight, 500),
                new IntakeDrive(follower, path4, intake, 500),
                new SpoolDriveShoot(follower, path5, launcher, turret, intake, limelight, 500),
                new IntakeDrive(follower, path6, intake, 500),
                new SpoolDriveShoot(follower, path7, launcher, turret, intake, limelight, 500),
                new FollowPathCommand(follower, path8),
                new SavePoseCommand(follower)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void initialize_loop(){
        telemetry.addLine("OpMode selected");
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
