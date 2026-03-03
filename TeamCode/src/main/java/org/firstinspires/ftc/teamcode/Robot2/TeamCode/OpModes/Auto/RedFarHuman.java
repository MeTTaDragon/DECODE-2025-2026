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

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.IntakeDrive;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.SpoolDriveShoot;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.MixedShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolUpCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;

public class RedFarHuman extends CommandOpMode {
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

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.000, 8.500),

                                    new Pose(89, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89, 10),

                                    new Pose(132.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.000, 8.500),

                                    new Pose(89, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89, 10),

                                    new Pose(132.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.000, 8.500),

                                    new Pose(89, 10)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89, 10),

                                    new Pose(105.000, 17.000)
                            )
                    ).setConstantHeadingInterpolation( Math.toRadians(0))

                    .build();
        }
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

                new SavePoseCommand(follower),
                // 1. Launch Preload immediately on start
                new SpoolUpCommand(launcher, limelight),
                new WaitCommand(1000),
                new ShootCommand(launcher, limelight, turret, intake, true),
                new WaitCommand(800),
                new StopLaunchCommand(launcher, turret, intake, limelight),

                // 2. Go to pickup spike
                new IntakeDrive(follower, paths.Path1, intake, 400),

                //3. Go shoot man
                new SpoolDriveShoot(follower, paths.Path2, launcher, turret, intake, limelight, true,500),


                //4. Go pick up from human man
                new IntakeDrive(follower, paths.Path3, intake, 400),


                //5. Go shoot again man
                new SpoolDriveShoot(follower, paths.Path4, launcher, turret, intake, limelight, true,500),


                //6. Go human player again man
                new IntakeDrive(follower, paths.Path5, intake, 400),


                //7. Go shoot again man
                new SpoolDriveShoot(follower, paths.Path6, launcher, turret, intake, limelight, true,500),


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