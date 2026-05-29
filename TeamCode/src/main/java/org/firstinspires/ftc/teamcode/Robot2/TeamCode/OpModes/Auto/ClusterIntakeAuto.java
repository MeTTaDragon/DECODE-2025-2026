package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.Alliance;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.AutoMixedShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.AutoStopLaunchCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.LimelightBallFollowCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands.LimelightScanCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.CheckLoadCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.DynamicPathFollowCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.LimelightModeCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SavePoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolUpCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;

/**
 * Cluster Intake Auto — Concept opmode for BioBuzz practice.
 *
 * Start position: center of field, intake facing the low-Y wall (away from goals).
 * The robot scans by spinning until the Limelight detects a ball cluster, drives
 * toward it while intaking, then navigates to a fixed shooting pose and fires.
 * The cycle repeats; FTC's 30-second autonomous timer ends the match naturally.
 *
 * Before running:
 *   1. Set SHOOT_POSE_X / SHOOT_POSE_Y for your alliance in FTC Dashboard.
 *      Blue default: (54, 108)   Red default: (90, 108)
 *   2. Confirm LimelightSubsystem.BALL_SCAN_PIPELINE matches your color pipeline index.
 *   3. Tune LimelightScanCommand and LimelightBallFollowCommand constants via Dashboard.
 */
@Config
@Autonomous(group = "concept", name = "Cluster Intake Auto")
public class ClusterIntakeAuto extends CommandOpMode {

    // Shooting pose — adjust per alliance. Blue: ~(54,108), Red: ~(90,108)
    public static double SHOOT_POSE_X = 54;
    public static double SHOOT_POSE_Y = 108;
    public static double SHOOT_HEADING_DEG = 180;

    // How long to hold the shoot state before resetting (ms)
    public static long SHOOT_HOLD_MS = 1000;

    private Follower follower;
    private Intake intake;
    private Launcher launcher;
    private Turret turret;
    private LimelightSubsystem limelight;
    private ColorSensor colorSensor;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));

    @Override
    public void initialize() {
        super.reset();

        alliance = Alliance.BLUE; // change to RED if needed

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        colorSensor = new ColorSensor(hardwareMap, "intrare");
        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower);

        register(turret, launcher, intake, limelight);

        schedule(buildAutoSequence());
    }

    private SequentialCommandGroup buildAutoSequence() {
        return new SequentialCommandGroup(
                new TurretStateCommand(turret, Turret.TurretState.MIXED),

                buildCycle(),
                buildCycle(),
                buildCycle()
        );
    }

    /**
     * One full cycle: scan → intake → shoot.
     */
    private SequentialCommandGroup buildCycle() {
        Pose shootPose = new Pose(SHOOT_POSE_X, SHOOT_POSE_Y, Math.toRadians(SHOOT_HEADING_DEG));
        double shootHeadingRad = Math.toRadians(SHOOT_HEADING_DEG);

        return new SequentialCommandGroup(

                // — SCAN —
                // Switch to ball-detection pipeline, then spin until cluster found (or 3.5s)
                new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BALL_SCAN),
                new LimelightScanCommand(follower)
                        .raceWith(new WaitCommand(3500)),

                // — INTAKE —
                // Drive toward cluster (proportional heading correction via tx) while intaking.
                // Race ends on: ball loaded (CheckLoadCommand) | hard timeout | cluster lost.
                new ParallelRaceGroup(
                        new LimelightBallFollowCommand(follower),
                        new CheckLoadCommand(colorSensor),
                        new IntakeStateCommand(intake, Intake.IntakeState.INTAKE),
                        new WaitCommand(3000)
                ),
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE),

                // — SHOOT —
                // Start spooling while driving to the shooting pose, then fire.
                new SpoolUpCommand(launcher, limelight),
                new DynamicPathFollowCommand(follower, shootPose, shootHeadingRad),
                new SavePoseCommand(follower),
                new WaitCommand(400),
                new AutoMixedShootCommand(launcher, turret, intake),
                new WaitCommand(SHOOT_HOLD_MS),
                new AutoStopLaunchCommand(launcher, turret, intake, limelight)
        );
    }

    @Override
    public void initialize_loop() {
        telemetry.addLine("Cluster Intake Auto — ready");
        telemetry.addData("Shoot pose", "(" + SHOOT_POSE_X + ", " + SHOOT_POSE_Y + ")");
    }

    @Override
    public void run() {
        super.run();

        follower.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("llta", llta);
        telemetry.addData("lltx", lltx);
        telemetry.addData("Target Vel", launcher.getTargetVelocity());
        telemetry.addData("Current Vel", launcher.getVelocity());
        telemetry.update();
    }
}
