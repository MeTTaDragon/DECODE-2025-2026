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
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperOpen;

@Autonomous(name = "Red close 12 ball-open gate after preload", group = "Auto red")
public class RedClose12BallOpenGateAfterPreload extends CommandOpMode {
    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;


    private Follower follower;

    // MIRROR CALCULATION (X only):
    // Blue X: 27.000 -> Red X: 144 - 27 = 117.000
    // Y remains 126.534
    // Heading 180 (Left) -> 0 (Right)
    private final Pose startPose = new Pose(117.000, 126.534, Math.toRadians(0));

    // Instance of the new Paths class
    private Paths paths;

    // --- Inner Class for Paths ---
    public static class Paths {
        public PathChain Path1; public PathChain Path2; public PathChain Path3;
        public PathChain Path4; public PathChain Path5; public PathChain Path6;
        public PathChain Path7; public PathChain Path8; public PathChain Path9;

        public Paths(Follower follower) {
            // Path 1: Preload
            // Blue: (27, 126.5) -> (62, 84)
            // Red X: 144-27=117 -> 144-62=82. Y stays same.
            Path1 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(117.000, 126.534), new Pose(88.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 2: Back off for approach
            // Blue: (62, 84) -> (21, 84.5)
            // Red X: 144-62=82 -> 144-21=123.
            Path2 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(88.000, 84.000), new Pose(123.000, 84.500))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 3: Curve to Intake Sample 1
            // Blue Control: (25, 69.5) -> Red X: 144-25=119
            // Blue End: (15, 70) -> Red X: 144-15=129
            Path3 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(123.000, 84.500), new Pose(119.000, 71), new Pose(129.000, 71.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 4: Go to Score Sample 1
            // Red End X: 144-62=82
            Path4 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(129.000, 71.000), new Pose(88.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 5: Curve to Intake Sample 2
            // Blue Control: (62, 57) -> Red X: 144-62=82
            // Blue End: (20, 59) -> Red X: 144-20=124
            Path5 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(88.000, 84.000), new Pose(88.000, 57.000), new Pose(124.000, 59.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 6: Go to Score Sample 2
            Path6 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(124.000, 59.000), new Pose(88.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 7: Curve to Intake Sample 3
            // Blue Control: (62, 32) -> Red X: 144-62=82
            // Blue End: (21, 35) -> Red X: 144-21=123
            Path7 = follower.pathBuilder().addPath(
                    new BezierCurve(new Pose(88.000, 84.000), new Pose(88.000, 32.000), new Pose(123.000, 35.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 8: Go to Score Sample 3
            Path8 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(123.000, 35.000), new Pose(88.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();

            // Path 9: Park
            // Blue End: (21, 84) -> Red X: 144-21=123
            Path9 = follower.pathBuilder().addPath(
                    new BezierLine(new Pose(88.000, 84.000), new Pose(123.000, 84.000))
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
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
                new ParallelCommandGroup(
                        new InstantCommand(() ->intakeState(Intake.IntakeState.IDLE)),
                        new InstantCommand(() ->launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING)),
                        new InstantCommand(() ->turret.setTurretState(Turret.TurretState.FULL_PINPOINT)),
                        new InstantCommand(() -> limelight.setMode(LimelightSubsystem.LimelightMode.BASKET))
                ),
                new WaitUntilCommand(() -> launcher.isVelocityReached()),
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.FULL_LIMELIGHT)),
                new InstantCommand(() -> launcher.setStopperPose(Launcher.stopperOpen)),
                new WaitCommand(650),
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE))
        );
    }

    private Command stopLaunchSequence() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> {
                    launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE);
                    launcher.setStopperPose(Launcher.stopperClose);
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
        alliance = Alliance.RED;
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
                new FollowPathCommand(follower, paths.Path2),
                // Turn on intake during the curve approach (Path3)
                //intakeState(Intake.IntakeState.IDLE),
                new FollowPathCommand(follower, paths.Path3),
                savePoseCommand(),
                //intakeState(Intake.IntakeState.IDLE), // Ensure hold
                new WaitCommand(325),

                new FollowPathCommand(follower, paths.Path4), // Score Sample 1
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                // --- Sample 2 ---
                new FollowPathCommand(follower, paths.Path5),
                savePoseCommand(),
                new WaitCommand(500),
                //intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, paths.Path6), // Score Sample 2
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                // --- Sample 3 ---
                new FollowPathCommand(follower, paths.Path7),
                savePoseCommand(),
                new WaitCommand(700),

                //intakeState(Intake.IntakeState.IDLE),

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