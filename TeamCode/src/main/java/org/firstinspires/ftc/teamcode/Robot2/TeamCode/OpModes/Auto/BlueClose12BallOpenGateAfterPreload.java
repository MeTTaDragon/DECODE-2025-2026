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

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

@Autonomous(name = "Blue close 12 ball-open gate after preload", group = "Auto")
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
                    new BezierLine(new Pose(27.000, 126.534), new Pose(62.000, 84.000))
            ).setConstantHeadingInterpolation( Math.toRadians(180)).build();

            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(62.000, 84.000), new Pose(21.000, 84.500))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(21.000, 84.500), new Pose(25.000, 69.5), new Pose(15, 70))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(15, 70), new Pose(62.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(62.000, 84.000), new Pose(62.000, 57.000), new Pose(20, 59))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(20, 59), new Pose(62.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(62.000, 84.000), new Pose(62.000, 32.000), new Pose(21.000, 35.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(21.000, 35.000), new Pose(62.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(62.000, 84.000), new Pose(21.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
        }
    }

    // --- Helper Methods ---
    private InstantCommand setLauncherState(Launcher.LauncherState state) {
        return new InstantCommand(() -> launcher.setCurrentLauncherState(state), launcher);
    }

    private InstantCommand setTurretState(Turret.TurretState state) {
        return new InstantCommand(() -> turret.setTurretState(state), turret);
    }

    private InstantCommand setLimelightMode(LimelightSubsystem.LimelightMode mode) {
        return new InstantCommand(() -> limelight.setMode(mode), limelight);
    }

    private InstantCommand setStopperPose(double pose) {
        return new InstantCommand(() -> launcher.setStopperPose(pose), launcher);
    }

    private InstantCommand intakeState(Intake.IntakeState state) {
        return new InstantCommand(() -> intake.setIntakeState(state), intake);
    }

    // --- Sequences ---
    private Command launchSequence() {
        return new SequentialCommandGroup(
                // Spin up launcher and aim turret simultaneously
                new ParallelCommandGroup(
                        setLauncherState(Launcher.LauncherState.SHOOTING),
                        setTurretState(Turret.TurretState.FULL_PINPOINT)
                ),
                // Wait for flywheel velocity
                new WaitUntilCommand(() -> launcher.isVelocityReached()),
                // Targeting
                setLimelightMode(LimelightSubsystem.LimelightMode.BASKET),
                setTurretState(Turret.TurretState.FULL_LIMELIGHT),
                // Release the stopper
                setStopperPose(stopperOpen),
                // Wait for stopper to clear
                new WaitCommand(500),
                // Feed the balls
                intakeState(Intake.IntakeState.REVERSE)
        );
    }

    private Command stopLaunchSequence() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> {
                    launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE);
                    launcher.setStopperPose(stopperClose);
                }, launcher),

                setTurretState(Turret.TurretState.IDLE),
                intakeState(Intake.IntakeState.IDLE),
                setLimelightMode(LimelightSubsystem.LimelightMode.PAUSE)
        );
    }

    private Command savePoseCommand() {
        return new InstantCommand(() ->
                lastAutoPose = follower.getPose()
        );
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
                new FollowPathCommand(follower, paths.Path1),
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                // --- Sample 1 ---
                // New Trajectory split: Path2 (Approach) + Path3 (Curve to Intake)
                new FollowPathCommand(follower, paths.Path2),
                // Turn on intake during the curve approach (Path3)
                intakeState(Intake.IntakeState.IDLE), // Assuming IDLE means ON based on your code context, or change to FORWARD if needed
                new FollowPathCommand(follower, paths.Path3),
                savePoseCommand(),
                new WaitCommand(325),
                intakeState(Intake.IntakeState.IDLE), // Ensure hold

                new FollowPathCommand(follower, paths.Path4), // Score Sample 1
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                // --- Sample 2 ---
                // Path5 handles the full curve from Score to Intake 2
                new FollowPathCommand(follower, paths.Path5),
                savePoseCommand(),
                new WaitCommand(500),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, paths.Path6), // Score Sample 2
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                // --- Sample 3 ---
                // Path7 handles the curve from Score to Intake 3
                new FollowPathCommand(follower, paths.Path7),
                savePoseCommand(),
                new WaitCommand(700),

                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, paths.Path8), // Score Sample 3
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),

                // --- Park ---
                new FollowPathCommand(follower, paths.Path9),
                savePoseCommand()
        );

        schedule(autonomousSequence);
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