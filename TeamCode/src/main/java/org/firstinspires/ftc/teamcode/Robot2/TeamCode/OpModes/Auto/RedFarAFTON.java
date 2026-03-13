package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.Alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.brakingpower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.IntakeDrive;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.SpoolDriveShoot;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.LauncherStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.MixedShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolUpCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;

@Autonomous(group = "redfar", name = "red far afton")

public class RedFarAFTON extends CommandOpMode {
    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;

    private Follower follower;

    // MIRROR CALCULATION:
    // Blue X: 65.000 -> Red X: 144 - 65 = 79.000
    // Y remains 7.5
    // Heading 180 (Left) -> 0 (Right)
    private final Pose startPose = new Pose(83.000, 7.5, Math.toRadians(0));

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
                            new BezierLine(
                                    new Pose(83, 7.5),

                                    new Pose(132.000, 8.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(132, 8.5),
                                    new Pose(125, 8.5)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toDegrees(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(125, 8.5),

                                    new Pose(132.000, 8.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setGlobalDeceleration(1.4)
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.000, 17),

                                    new Pose(89, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setGlobalDeceleration(brakingpower)
                    .build();


            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89, 10),

                                    new Pose(105.000, 17.000)
                            )
                    ).setConstantHeadingInterpolation( Math.toRadians(0))
                    .setGlobalDeceleration(brakingpower)
                    .build();
        }
    }


    ElapsedTime timer;


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

        timer = new ElapsedTime();

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(

                new InstantCommand(() -> timer.reset()),

                new TurretStateCommand(turret, Turret.TurretState.MIXED),
                new SavePoseCommand(follower),
                // 1. Launch Preload immediately on start
                new SpoolUpCommand(launcher, limelight),
                new WaitCommand(1000),
                new MixedShootCommand(launcher, turret, intake),
                new WaitCommand(800),
                new StopLaunchCommand(launcher, turret, intake, limelight),
                new LauncherStateCommand(launcher, Launcher.LauncherState.IDLE),

                new WaitUntilCommand(() -> timer.seconds() > 10),

                // 2. Go to pickup spike
                new IntakeDrive(follower, 0.7, paths.Path1, intake, 400),

                new FollowPathCommand(follower, paths.Path2),

                new WaitUntilCommand(() -> timer.seconds() > 31),

                new SpoolUpCommand(launcher, limelight),
                new WaitCommand(1000),
                new MixedShootCommand(launcher, turret, intake),
                new WaitCommand(800),
                new StopLaunchCommand(launcher, turret, intake, limelight),
                new LauncherStateCommand(launcher, Launcher.LauncherState.IDLE),

                //8. leave launch zone man
                new FollowPathCommand(follower, paths.Path7),
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