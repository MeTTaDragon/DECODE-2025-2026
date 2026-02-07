package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.Alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.lastAutoPose;

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

@Autonomous(name = "Blue Far Spike and Human x2", group = "Auto")
public class BlueFarSpikeAndHuman extends CommandOpMode {
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
    private final Pose startPose = new Pose(63, 7.5, Math.toRadians(180));

    private Paths paths;

    // --- Inner Class for Paths ---

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(63, 7.500),
                                    new Pose(72.214, 37.537),
                                    new Pose(22.000, 35.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 35.500),

                                    new Pose(54, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54, 10),

                                    new Pose(8.500, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.500, 8.500),

                                    new Pose(54, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54, 10),

                                    new Pose(8.500, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.500, 8.500),

                                    new Pose(54, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54, 10),

                                    new Pose(39.000, 17.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

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
        return new InstantCommand(() -> lastAutoPose = follower.getPose());
    }

    @Override
    public void initialize() {
        super.reset();
        // CHANGED TO RED
        alliance = Alliance.BLUE;

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

                savePoseCommand(),
                // 1. Launch Preload immediately on start
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                // 2. Go to pickup spike
                intakeState(Intake.IntakeState.REVERSE),
                new FollowPathCommand(follower, paths.Path1),
                new WaitCommand(200),
                //intakeState(Intake.IntakeState.IDLE),
                //3. Go shoot man
                new FollowPathCommand(follower, paths.Path2),
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                //3. Go pick up from human man
                intakeState(Intake.IntakeState.REVERSE),
                new FollowPathCommand(follower, paths.Path3),
                savePoseCommand(),
                new WaitCommand(200),
                //intakeState(Intake.IntakeState.IDLE),
               //4. Go shoot again man
                new FollowPathCommand(follower, paths.Path4),
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                //5. Go human player again man
                intakeState(Intake.IntakeState.REVERSE),
                new FollowPathCommand(follower, paths.Path5),
                savePoseCommand(),
                new WaitCommand(200),
                //intakeState(Intake.IntakeState.IDLE),
                //6. Go shoot again man
                new FollowPathCommand(follower, paths.Path6),
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                //7. leave launch zone man
                new FollowPathCommand(follower, paths.Path7),
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