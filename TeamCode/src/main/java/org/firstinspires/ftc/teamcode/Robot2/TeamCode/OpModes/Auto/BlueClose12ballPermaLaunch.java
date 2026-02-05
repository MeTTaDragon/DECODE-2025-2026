package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.Alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.imuHeading;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.lastAutoPose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.llRx;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.llRy;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.llta;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.lltx;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.llty;

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

@Autonomous(name = "Blue close 12 ball perma launch", group = "Auto")
public class BlueClose12ballPermaLaunch extends CommandOpMode {
    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

     // Ensure this is defined
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
                .addPath(new BezierLine(new Pose(55, 84.000), new Pose(21.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(21.000, 84.000), new Pose(55, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // Path 4
        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(55, 84.000), new Pose(73.000, 55.000), new Pose(25.000, 58.000)))
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
                        //setLauncherState(Launcher.LauncherState.SHOOTING),
                        //setLimelightMode(LimelightSubsystem.LimelightMode.BASKET),
                        setTurretState(Turret.TurretState.FULL_PINPOINT)
                ),
                // Wait for flywheel velocity
                new WaitUntilCommand(() -> launcher.isVelocityReached()),
                // Targeting
                //setTurretState(Turret.TurretState.FULL_LIMELIGHT),
                // Release the stopper
                setStopperPose(Launcher.stopperOpen),
                // Wait for stopper to clear
                new WaitCommand(800),
                // Feed the balls
                intakeState(Intake.IntakeState.REVERSE)
        );
    }

    private Command stopLaunchSequence() {
        return new ParallelCommandGroup(
                // FIXED: Combined Launcher actions into one command
                new InstantCommand(() -> {
                    //launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE);
                    launcher.setStopperPose(Launcher.stopperClose);
                }, launcher),

                setTurretState(Turret.TurretState.IDLE),
                intakeState(Intake.IntakeState.IDLE),
                setLimelightMode(LimelightSubsystem.LimelightMode.PAUSE)
        );
    }
    private Command stopperPose(double x) {
        return new InstantCommand(() -> {
            launcher.setStopperPose(x);
        }, launcher);
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

        buildPaths();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new InstantCommand(() -> Launcher.useLimelight = false),
                new InstantCommand(() -> launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING)),
                new FollowPathCommand(follower, path1),
                savePoseCommand(),
                launchSequence(),
                new WaitCommand(1800),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                new FollowPathCommand(follower, path2),
                savePoseCommand(),
                new WaitCommand(500),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, path3),
                savePoseCommand(),
                new WaitCommand(300),
                launchSequence(),
                new WaitCommand(500),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                new FollowPathCommand(follower, path4),
                savePoseCommand(),
                new WaitCommand(500),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, path4_1),
                savePoseCommand(),
                new WaitCommand(325),

                new FollowPathCommand(follower, path5),
                savePoseCommand(),
                new WaitCommand(300),
                launchSequence(),
                new WaitCommand(700),
                stopLaunchSequence(),
                intakeState(Intake.IntakeState.REVERSE),

                new FollowPathCommand(follower, path6),
                savePoseCommand(),
                new WaitCommand(500),
                intakeState(Intake.IntakeState.IDLE),

                new FollowPathCommand(follower, path7, 0.9),
                savePoseCommand(),
                new WaitCommand(300),
                launchSequence(),
                new WaitCommand(500),
                stopLaunchSequence(),

                new FollowPathCommand(follower, path8, 0.9),
                savePoseCommand(),
                new InstantCommand(() -> launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE))
        );

        schedule(autonomousSequence);
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Current velocity", launcher.getVelocity());
        telemetry.addData("Target velocity", launcher.getTargetVelocity());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
        telemetry.addData("turret target heading", Math.toDegrees(turret.getTargetHeading()));
        telemetry.addData("distance", launcher.getDistance());
        telemetry.addData("alliance", alliance);
        telemetry.addData("limelight mode", limelight.getCurrentMode());
        telemetry.addData("ta", llta);
        telemetry.addData("tx", lltx);
        telemetry.addData("ty", llty);
        telemetry.addData("llRx", llRx);
        telemetry.addData("llRy", llRy);
        telemetry.addData("imu heading", imuHeading);
        telemetry.update();
    }
}