package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.Alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.IntakeDrive;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.SpoolDriveShoot;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
@Config
@Autonomous(group = "redfar",name="red 18")
public class Red18 extends CommandOpMode {

    Intake intake;
    Launcher launcher;
    Turret turret;
    LimelightSubsystem limelight;
    ColorSensor colorSensor;
    private Follower follower;

    private final Pose startPose = new Pose(117, 127.5, Math.toRadians(0));
    public static class Paths {
        public PathChain Launch1;
        public PathChain IntakeMid;
        public PathChain OpenGate;
        public PathChain Launch2;
        public PathChain Recycle;
        public PathChain Launch3;
        public PathChain Recycle2;
        public PathChain Launch4;
        public PathChain Recycle3;
        public PathChain Launch5;
        public PathChain CloseLine;
        public PathChain Launch6;

        public Paths(Follower follower) {
            Launch1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117, 127.5),

                                    new Pose(85, 85.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(Constants.pathConstraints.getBrakingStrength()).setBrakingStart(Constants.pathConstraints.getBrakingStart()).setGlobalDeceleration(brakingpower)
                    .build();

            IntakeMid = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85, 85.000),
                                    new Pose(89.622, 39.512),
                                    new Pose(130, 60)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            new BezierCurve(
                                    new Pose(130, 60.000),
                                    new Pose(115.622, 64.476),
                                    new Pose(127.3, 69.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();


            Launch2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.3, 69.5),

                                    new Pose(97, 87.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(Constants.pathConstraints.getBrakingStrength()).setBrakingStart(Constants.pathConstraints.getBrakingStart()).setGlobalDeceleration(brakingpower)
                    .build();

            Recycle = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(97, 87.000),
                                    new Pose(93, 62.5),

                                    new Pose(127, 68.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .addParametricCallback(1, () -> new WaitCommand(300))
                    .addPath(
                            new BezierCurve(
                                    new Pose(127, 68.000),
                                    new Pose(122, 57),
                                    new Pose(131, 48.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .setBrakingStrength(Constants.pathConstraints.getBrakingStrength()).setBrakingStart(Constants.pathConstraints.getBrakingStart()).setGlobalDeceleration(brakingpower)
                    .build();

            Launch3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131, 48),
                                    new Pose(99.3, 56.8),

                                    new Pose(97, 87.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .setBrakingStrength(Constants.pathConstraints.getBrakingStrength()).setBrakingStart(Constants.pathConstraints.getBrakingStart()).setGlobalDeceleration(brakingpower)
                    .build();

            CloseLine = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97, 87.000),
                                    new Pose(127, 84.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setGlobalDeceleration(brakingpower)
                    .setVelocityConstraint(0.9)
                    .build();

            Launch4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127, 84),

                                    new Pose(97, 87.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setVelocityConstraint(0.9)
                    .setGlobalDeceleration(brakingpower)
                    .build();


            Launch5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131, 48),

                                    new Pose(90, 111.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .setVelocityConstraint(0.9)
                    .setGlobalDeceleration(brakingpower)
                    .build();



            Launch6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131, 48),

                                    new Pose(97, 87)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .setGlobalDeceleration(brakingpower)
                    .build();
        }
    }



    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.RED;

        // Initialize Follower and Subsystems
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        colorSensor = new ColorSensor(hardwareMap, "intrare");
        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower);

        register(turret, launcher, intake, limelight);

        Paths paths = new Paths(follower);

        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new TurretStateCommand(turret, Turret.TurretState.MIXED),

                new SpoolDriveShoot(follower, paths.Launch1, launcher, turret, intake, limelight, 1000),

                new IntakeDrive(follower, paths.IntakeMid, intake, colorSensor,100),

                new SpoolDriveShoot(follower, paths.Launch2, launcher, turret, intake, limelight, 1000),

                new IntakeDrive(follower, paths.Recycle, intake, colorSensor,1000),

                new SpoolDriveShoot(follower, paths.Launch3, launcher, turret, intake, limelight, 1000),

                new IntakeDrive(follower, paths.CloseLine, intake, colorSensor,300),

                new SpoolDriveShoot(follower, paths.Launch4, launcher, turret, intake, limelight, 1000),

                new IntakeDrive(follower, paths.Recycle, intake, colorSensor,1000),

                new SpoolDriveShoot(follower, paths.Launch5, launcher, turret, intake, limelight, 1000),

                new IntakeDrive(follower, paths.Recycle, intake, colorSensor,1000),

                new SpoolDriveShoot(follower, paths.Launch6, launcher, turret, intake, limelight, 1000)
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
        telemetry.addData("Target Vel", launcher.getTargetVelocity());
        telemetry.addData("Current Vel", launcher.getVelocity());

        telemetry.update();
    }
}