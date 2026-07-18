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

@Autonomous(group = "blueclose", name = "red close no gate test")
public class BlueCloseGateAfterPre extends CommandOpMode {

    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.65;
    private Follower follower;

    // Define Poses cleanly at the top level
    private final Pose startPose = new Pose(107.766, 135.987, Math.toRadians(0));
    private final Pose shooting_pose = new Pose(83.603, 93.453, Math.toRadians(0));
    private final Pose spike1 = new Pose(119.702, 82.461, Math.toRadians(0));
    private final Pose spike2 = new Pose(111.1, 65.03, Math.toRadians(0));
    private final Pose spike3 = new Pose(119.19, 41.600, Math.toRadians(0));
    private final Pose exit_zone = new Pose(102.37, 81.48, Math.toRadians(0));

    // Control Points for Curves
    private final Pose ctrl_spike1 = new Pose(121, 90.57);
    private final Pose ctrl_spike2 = new Pose(82.23, 65.000);
    private final Pose ctrl_spike3 = new Pose(71.390, 39.990);

    private PathChain shoot_pre, pickup_spike1, shoot_spike1, pickup_spike2, shoot_spike2, pickup_spike3, shoot_spike3, leave_zone;

    public void buildPaths() {
        // Path 1
        shoot_pre = follower.pathBuilder()
                .addPath(new BezierLine(startPose.mirror(144), shooting_pose.mirror(144)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 2
        pickup_spike1 = follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose.mirror(144), ctrl_spike1.mirror(144), spike1.mirror(144)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 3
        shoot_spike1 = follower.pathBuilder()
                .addPath(new BezierLine(spike1.mirror(144), shooting_pose.mirror(144)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 4
        pickup_spike2 = follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose.mirror(144), ctrl_spike2.mirror(144), spike2.mirror(144)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 5
        shoot_spike2 = follower.pathBuilder()
                .addPath(new BezierLine(spike2.mirror(144), shooting_pose.mirror()))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 6
        pickup_spike3 = follower.pathBuilder()
                .addPath(new BezierCurve(shooting_pose.mirror(), ctrl_spike3.mirror(), spike3.mirror()))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 7
        shoot_spike3 = follower.pathBuilder()
                .addPath(new BezierLine(spike3.mirror(), shooting_pose.mirror()))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path 8
        leave_zone = follower.pathBuilder()
                .addPath(new BezierLine(shooting_pose.mirror(), exit_zone.mirror()))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.BLUE;

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
                new SpoolDriveShoot(follower, shoot_pre, launcher, turret, intake, limelight, 500),
                new IntakeDrive(follower, pickup_spike1, intake, 500),
                new SpoolDriveShoot(follower, shoot_spike1, launcher, turret, intake, limelight, 500),
                new IntakeDrive(follower, pickup_spike2, intake, 500),
                new SpoolDriveShoot(follower, shoot_spike2, launcher, turret, intake, limelight, 500),
                new IntakeDrive(follower, pickup_spike3, intake, 500),
                new SpoolDriveShoot(follower, shoot_spike3, launcher, turret, intake, limelight, 500),
                new FollowPathCommand(follower, leave_zone),
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
