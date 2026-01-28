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

@Autonomous(name = "Blue close 12 ball", group = "Auto")
public class BlueClose12ball extends CommandOpMode {

    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.65; // Ensure this is defined
    private Follower follower;

    private final Pose startPose = new Pose(27.000, 126.534, Math.toRadians(180));
    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    public void buildPaths() {
        // Path 1
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(27.000, 126.534), new Pose(62.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 2
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(62.000, 84.000), new Pose(24.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.000, 84.000), new Pose(62.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(62.000, 84.000), new Pose(73.000, 55.000), new Pose(24.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 5
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.000, 60.000), new Pose(62.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 6
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(62.000, 84.000), new Pose(72.000, 32.000), new Pose(24.000, 36.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 7
        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.000, 36.000), new Pose(62.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 8
        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(62.000, 84.000), new Pose(32.854, 82.732)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
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
    public Command launchSequence() {
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
                // Release the stopper
                setStopperPose(stopperOpen),
                // Wait for stopper to clear
                new WaitCommand(500),
                // Feed the balls
                intakeState(Intake.IntakeState.REVERSE)
        );
    }

    public Command stopLaunchSequence() {
        return new ParallelCommandGroup(
                // FIXED: Combined Launcher actions into one command
                new InstantCommand(() -> {
                    launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE);
                    launcher.setStopperPose(stopperClose);
                }, launcher),

                setTurretState(Turret.TurretState.IDLE),
                intakeState(Intake.IntakeState.IDLE),
                setLimelightMode(LimelightSubsystem.LimelightMode.PAUSE)
        );
    }

    @Override
    public void initialize() {
        super.reset();

        // Initialize Follower and Subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, path1),
                launchSequence(),
                new WaitCommand(2000),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                new FollowPathCommand(follower, path2),
                new WaitCommand(300),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, path3),
                launchSequence(),
                new WaitCommand(2000),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                new FollowPathCommand(follower, path4),
                new WaitCommand(300),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, path5),
                launchSequence(),
                new WaitCommand(2000),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                new FollowPathCommand(follower, path6),
                new WaitCommand(300),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, path7),
                launchSequence(),
                new WaitCommand(2000),
                stopLaunchSequence(),

                new FollowPathCommand(follower, path8)
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        if (follower.getPose().getY() < launcher.middle_Y) {
            launcher.setHoodPose(launcher.farHoodPose);
        } else {
            launcher.setHoodPose(launcher.closeHoodPose);
        }

        launcher.targetVelocity = Math.pow(launcher.getDistance(), 0.4768327) * 183.7126 + 125; //de ce +100? -R: pt ca launcher ul nu atinge velocity ul si calculul nu e 100% precise. E nevoie de un supliment-Alda
         follower.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }
}