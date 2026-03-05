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

public class BlueClose12BallOpenGateAfterPreload extends CommandOpMode {
    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.65;
    private Follower follower;

    private final Pose startPose = new Pose(27.000, 126.534, Math.toRadians(180));

    // Instance of the new Paths class
    private Paths paths;

    // --- Inner Class for Paths (From Snippet) ---
    public static class Paths {
        public PathChain Path1; public PathChain Path2; public PathChain Path3;
        public PathChain Path4; public PathChain Path5; public PathChain Path6;
        public PathChain Path7; public PathChain Path8; public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(27.000, 126.534), new Pose(55, 84.000))
            ).setConstantHeadingInterpolation( Math.toRadians(180)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(55, 84.000), new Pose(17, 84.500))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(17.000, 84.500), new Pose(25.000, 69.5), new Pose(15, 70))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(15, 70), new Pose(55, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(55, 84.000), new Pose(55, 57.000), new Pose(20, 59))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(20, 59), new Pose(55, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(55, 84.000), new Pose(55, 32.000), new Pose(21.000, 35.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(21.000, 35.000), new Pose(55, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(55, 84.000), new Pose(21.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
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

        // Initialize the new Paths object
        paths = new Paths(follower);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                // --- Preload ---
                new SpoolDriveShoot(follower, paths.Path1, launcher, turret, intake, limelight, 500),

                // --- Sample 1 ---
                // New Trajectory split: Path2 (Approach) + Path3 (Curve to Intake)
                new FollowPathCommand(follower, paths.Path2),
                // Turn on intake during the curve approach (Path3)
                new IntakeDrive(follower, paths.Path3, intake, 500),

                // --- Shoot 1 ---
                new SpoolDriveShoot(follower, paths.Path4, launcher, turret, intake, limelight, 500),

                // --- Sample 2 ---
                new IntakeDrive(follower, paths.Path5, intake, 500),

                // --- Shoot 2 ---
                new SpoolDriveShoot(follower, paths.Path6, launcher, turret, intake, limelight, 500),

                // --- Sample 3 ---
                new IntakeDrive(follower, paths.Path7, intake, 500),

                // --- Shoot 3 ---
                new SpoolDriveShoot(follower, paths.Path8, launcher, turret, intake, limelight, 500),

                // --- Park ---
                new FollowPathCommand(follower, paths.Path9),
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