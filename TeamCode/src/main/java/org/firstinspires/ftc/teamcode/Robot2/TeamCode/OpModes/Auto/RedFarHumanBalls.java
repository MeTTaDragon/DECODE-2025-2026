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

@Autonomous(name = "Red Far Human Balls", group = "Auto red")
public class RedFarHumanBalls extends CommandOpMode {
    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.65;
    private Follower follower;

    // MIRROR CALCULATION:
    // Blue X: 65.000 -> Red X: 144 - 65 = 79.000
    // Y remains 7.5
    // Heading 180 (Left) -> 0 (Right)
    private final Pose startPose = new Pose(83, 7.5, Math.toRadians(0));

    private Paths paths;

    // --- Inner Class for Paths ---
    public static class Paths {
        public PathChain Path0;
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path0 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(83, 7.500), new Pose(83, 10.0)
                    )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
            // Path 1: Go to Intake Position
            // Blue Start: (63, 7.5) -> Red X: 144-63 = 81
            // Blue Control: (61.854, 36.305) -> Red X: 144-61.854 = 82.146
            // Blue End: (21, 37.5) -> Red X: 144-21 = 123
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83, 10),
                                    new Pose(137, 22)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            // Path 2: Return to Shoot
            // Blue Start: (21, 37.5) -> Red X: 123
            // Blue End: (63, 7.5) -> Red X: 81
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137, 22),
                                    new Pose(137, 10.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-90))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137, 10.000),
                                    new Pose(89, 10.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            // Path 3: Park
            // Blue Start: (65, 9) -> Red X: 144-65 = 79
            // Blue End: (35, 10) -> Red X: 144-35 = 109
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89, 10.000),
                                    new Pose(109.000, 20.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();


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
        return new InstantCommand(() -> lastAutoPose = follower.getPose());
    }

    @Override
    public void initialize() {
        super.reset();
        // CHANGED TO RED
        alliance = Alliance.RED;

        // Initialize Follower and Subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower);

        register(turret, launcher, intake, limelight);

        // Initialize the Paths object
        paths = new Paths(follower);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.Path0),
                savePoseCommand(),
                // 1. Launch Preload immediately on start
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),

                // 2. Go to Intake Position (Path 1)
                intakeState(Intake.IntakeState.REVERSE),
                new FollowPathCommand(follower, paths.Path1),
                savePoseCommand(),
                new FollowPathCommand(follower, paths.Path2),
                savePoseCommand(),

                // 3. Intake On for 1.8 seconds
                // Assuming FORWARD intakes from field
                new WaitCommand(700),

                // 4. Return to Shoot (Path 2)
                new FollowPathCommand(follower, paths.Path3),
                savePoseCommand(),
                intakeState(Intake.IntakeState.IDLE),
                // 5. Launch Second Shot
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),

                // 6. Park (Path 3)
                new FollowPathCommand(follower, paths.Path4),
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