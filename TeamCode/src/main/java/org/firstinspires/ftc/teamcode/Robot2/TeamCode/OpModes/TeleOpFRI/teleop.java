package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.TeleOpFRI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.LauncherStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.LimelightModeCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.MixedShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.RumbleOnBallCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolStopper;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.SpoolUpCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopLaunchCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.GlobalsFRI;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
//import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.GlobalsFRI.*;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperClose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperOpen;

import java.util.List;

@Config
@TeleOp(name = "teleOp", group = "FRI")
public class teleop extends CommandOpMode {
    GamepadEx controller;
    GamepadEx spooler;

    // --- Robot Dimensions & LED Constants ---
    private final double ROBOT_WIDTH = 17.32;
    private final double ROBOT_LENGTH = 13.4;
    private final double LED_GREEN = 0.5;
    private final double LED_WHITE = 1.0;

    // --- Triangle Zone Coordinates (PEDRO PATHING COORDINATES: 0-144 inches) ---
    // You must change these numbers to match the actual field triangles

    // Front Triangle Corners (x, y)
    private final Pose2d frontA = new Pose2d(72, 72, 0);
    private final Pose2d frontB = new Pose2d(15, 128, 0);
    private final Pose2d frontC = new Pose2d(130, 128, 0);

    // Back Triangle Corners (x, y)
    private final Pose2d backA = new Pose2d(72, 23, 0);
    private final Pose2d backB = new Pose2d(100, 0, 0);
    private final Pose2d backC = new Pose2d(45, 0, 0);

    Follower follower;
    Servo ledAlliance;
    Servo ledShooter;

    Turret turret;
    Launcher launcher;
    Intake intake;
    LimelightSubsystem limelight;

    ColorSensor myColorSensor;

    List<LynxModule> allHubs;
    ElapsedTime timer;
    Gamepad.RumbleEffect customRumbleEffect;
    boolean rumbled = false;
    boolean shooterVelRumbled = false;
    private static double totallooptime = 0;
    private static double loops = 0;
    boolean shootOnFly = false;
    boolean mixedAim = true;
    public double DISTANCE_THRESHOLD_INCHES = 0.8;
    boolean isDetecting;
    boolean ballDetected;

    private ElapsedTime checkTimer;

    @Override
    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        checkTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        controller = new GamepadEx(gamepad1);
        spooler = new GamepadEx(gamepad2);

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 100)
                .build();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(lastAutoPose);

        super.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //telemetry.setMsTransmissionInterval(250);
        myColorSensor = new ColorSensor(hardwareMap, "intrare");
        ledAlliance = hardwareMap.get(Servo.class, "ledAlliance");
        ledShooter = hardwareMap.get(Servo.class, "ledShooter");
        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower, telemetry);



        register(launcher, turret, intake, limelight); // launcher before turret: SOF reads Launcher statics

        follower.startTeleopDrive(true);

        // MIXED: tintire pe odometrie + fine-trim din Limelight, cu constrangere mecanica corecta
        turret.setTurretState(Turret.TurretState.MIXED);
        limelight.init();
        limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);
        launcher.init();
        intake.init();



//        controller.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
//                new InstantCommand(() -> shootOnFly = !shootOnFly)
//        );
        controller.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    if (alliance == Alliance.RED) {
                        follower.setPose(new Pose(128, 79, Math.toRadians(0)));
                    } else {
                        follower.setPose(new Pose(16, 79, Math.toRadians(180)));
                    }
                } )
        );
        controller.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET);
                    follower.setPose(new Pose(llRx, llRy, follower.getHeading()));
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> launcher.add_comp -= 25)
        );
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> launcher.add_comp += 25)
        );
        //open stopper
        controller.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> launcher.setStopperPose(stopperOpen))
        );
        //close stopper
        controller.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> launcher.setStopperPose(stopperClose))
        );


        Trigger leftTrigger = new Trigger(() -> gamepad1.left_trigger > 0.1);

        // Left trigger: spool the flywheel (gamepad1 driver's job)
        leftTrigger.whileActiveOnce(
                new SpoolUpCommand(launcher, limelight)
        ).whenInactive(
                new SpoolStopper(launcher, intake)
        );

        // Right trigger: aim only — turret tracks goal + limelight, no spool/shoot
//        rightTrigger.whileActiveOnce(
//                new ConditionalCommand(
//                        new ParallelCommandGroup(
//                                new TurretStateCommand(turret, Turret.TurretState.SHOOT_ON_THE_FLY),
//                                new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET)
//                        ),
//                        new ParallelCommandGroup(
//                                new TurretStateCommand(turret, Turret.TurretState.MIXED),
//                                new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET)
//                        ),
//                        () -> shootOnFly
//                )
//        ).whenInactive(
//                new ParallelCommandGroup(
//                        new TurretStateCommand(turret, Turret.TurretState.IDLE),
//                        new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.PAUSE)
//                )
//        );
        // NOTE: gamepad1's right trigger no longer shoots — aiming/launching is exclusively gamepad2's job (see below).

        // ===================== GAMEPAD2 (spooler / shooter) =====================
        // LEFT_BUMPER: select FAR goal (persistent, doesn't require holding).
        // RIGHT_BUMPER: select NORMAL (close) goal (persistent).
        // Right trigger: shoot using whichever goal is currently selected.
        spooler.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    setGoalPose(farGoalPose);
                    limelight.setMode(LimelightSubsystem.LimelightMode.FAR_GOAL);
                })
        );
        spooler.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    setGoalPose(alliance == Alliance.RED ? redGoalPose : blueGoalPose);
                    limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);
                })
        );

        Trigger rightspoolerTrigger = new Trigger(() -> gamepad2.right_trigger > 0.1);

        rightspoolerTrigger.whileActiveOnce(
                new MixedShootCommand(launcher, turret, intake)
        ).whenInactive(
                new StopLaunchCommand(launcher, turret, intake, limelight)
        );
        // ==========================================================================


        //intake scuipa
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE), intake)
        ).whenReleased(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.IDLE), intake)
        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new IntakeStateCommand(intake, Intake.IntakeState.INTAKE)
        ).whenInactive(
                new IntakeStateCommand(intake, Intake.IntakeState.IDLE)
        );

        //reset position odometrie
        controller.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand(() -> {
                    follower.setPose(new Pose(72, 7.5, Math.toRadians(90)));
                    rumbled = false;
                })
        );
        //setare manuala alianta albastra
        controller.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(() -> {
                    alliance = Alliance.BLUE;
                    setGoalPose(blueGoalPose);
                })
        );
        //setare manuala alianta rosie
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new InstantCommand(() -> {
                    alliance = Alliance.RED;
                    setGoalPose(redGoalPose);
                })
        );


// Pass in the sensor and the gamepad you want to vibrate (e.g., gamepad1)
        RumbleOnBallCommand rumbleCommand = new RumbleOnBallCommand(myColorSensor, gamepad1, gamepad2);

// Schedule it so it starts running its execute() loop
        rumbleCommand.schedule();
    }

    /**
     * Sets the turret and launcher goal pose in one place, keeping targetGoalPose in sync.
     * Used by gamepad2's far/normal goal shooting triggers.
     */
    private void setGoalPose(Pose goal) {
        turret.goalPose = goal;
        turret.targetGoalPose = new Pose2d(goal.getX(), goal.getY(), 0);
        launcher.goalPose = goal;
    }

    /**
     * Updates the LED color based on Robot Position (checking intersection of corners AND edges) and Alliance
     */
    public void updateLEDs() {
        Pose currentPose = follower.getPose();

        // Check intersection with Front Triangle (Robot touching or inside)
        boolean touchingFront = isRobotTouchingTriangle(currentPose, frontA, frontB, frontC);

        // Check intersection with Back Triangle (Robot touching or inside)
        boolean touchingBack = isRobotTouchingTriangle(currentPose, backA, backB, backC);

        if (touchingFront || touchingBack) {
            // If ANY part touches, turn GREEN
            ledShooter.setPosition(LED_GREEN);
        } else {
            // Otherwise stay WHITE
            ledShooter.setPosition(LED_WHITE);
        }

        if(launcher.isVelocityReached() ) {
            ledAlliance.setPosition(LED_GREEN);
        }
        else {
            // Alliance Color Logic
            if (alliance == Alliance.RED) {
                ledAlliance.setPosition(0.28);
            } else if (alliance == Alliance.BLUE) {
                ledAlliance.setPosition(0.61);
            } else {
                ledAlliance.setPosition(0);
            }
        }

    }

    public void run() {
        timer.reset();
        follower.update();

        /*if(alliance == Alliance.RED){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        } else{
            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        }*/
//        if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0){
//            follower.holdPoint(follower.getPose());
//        }
//        else{
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        //}

        updateLEDs();

        if ((follower.getPose().getX() < 0 || follower.getPose().getX() > 144 || follower.getPose().getY() < 0 || follower.getPose().getY() > 144) && !rumbled) {
            rumbled = true;
            gamepad1.runRumbleEffect(customRumbleEffect);
        }

        // Rumble gamepad2 (shooter) once the launcher locks onto the correct velocity
        if (launcher.isVelocityReached()) {
            if (!shooterVelRumbled) {
                shooterVelRumbled = true;
                gamepad2.runRumbleEffect(customRumbleEffect);
            }
        } else {
            shooterVelRumbled = false;
        }

//        ballDetected = myColorSensor.distance(DistanceUnit.INCH) < DISTANCE_THRESHOLD_INCHES;
//
//        if (ballDetected) {
//            if (!isDetecting) {
//                // 1. The ball JUST arrived! Start the timer.
//                checkTimer.reset();
//                isDetecting = true;
//            } else if (checkTimer.milliseconds() > 150) {
//                intake.setintakePos(Intake.servoPosUp);
//            }
//        } else {
//            // 3. No ball is here, or the ball just left. Reset the timer and states!
//            isDetecting = false;
//        }

        telemetry.addData("ball detected", ballDetected);
        telemetry.addData("Current velocity", launcher.getVelocity());
        telemetry.addData("Target velocity", launcher.getTargetVelocity());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
        telemetry.addData("turret target heading", Math.toDegrees(turret.getTargetHeading()));
        telemetry.addData("distance", launcher.getDistance());
        telemetry.addData("alliance", alliance);
        telemetry.addData("SOF Mode", shootOnFly);
        telemetry.addData("SOF baseTargetVel", Launcher.baseTargetVelocity);
        telemetry.addData("SOF requiredSpeed", requiredSpeed);
        telemetry.addData("limelight mode", limelight.getCurrentMode());
        telemetry.addData("ta", llta);
        telemetry.addData("tx", lltx);
        telemetry.addData("llRx", llRx);
        telemetry.addData("llRy", llRy);
        telemetry.addData("Intake Target Vel " , intake.getTargetVelocity());
        telemetry.addData("Intake Current Vel " , intake.getCurrentVelocity());

        telemetry.addData("Loop Time", 1 / timer.seconds());
        totallooptime += 1 / timer.seconds();
        loops++;
        telemetry.addData("Average Looptime", totallooptime / loops);

        timer.reset();

        super.run();
        telemetry.update();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    // =========================================================================
    // ====================== GEOMETRY HELPER METHODS ==========================
    // =========================================================================

    /**
     * Master check: returns true if the Robot (Rectangle) overlaps with the Triangle
     * Checks both corners inside triangle AND edges crossing triangle edges.
     */
    private boolean isRobotTouchingTriangle(Pose robotPose, Pose2d tA, Pose2d tB, Pose2d tC) {
        // 1. Get the 4 corners of the robot
        Pose2d[] robotCorners = getRobotCorners(robotPose);

        // 2. Check if any Robot Corner is INSIDE the triangle
        for (Pose2d corner : robotCorners) {
            if (isPointInTriangle(corner, tA, tB, tC)) return true;
        }

        // 3. Check for Edge Intersections (The "Crossing" Case)
        // Robot Edges: [0-1], [1-3], [3-2], [2-0] (Indices based on getRobotCorners)
        // Triangle Edges: [A-B], [B-C], [C-A]

        Pose2d[] triCorners = {tA, tB, tC};

        // Indices for robot edges (0->1, 1->3, 3->2, 2->0)
        // Note: My corner generation is FL, FR, BL, BR.
        // So edges are: Front(0-1), Right(1-3), Back(3-2), Left(2-0).
        int[] rIdx = {0, 1, 3, 2};

        for (int i = 0; i < 4; i++) {
            Pose2d r1 = robotCorners[rIdx[i]];
            Pose2d r2 = robotCorners[rIdx[(i + 1) % 4]];

            for (int j = 0; j < 3; j++) {
                Pose2d t1 = triCorners[j];
                Pose2d t2 = triCorners[(j + 1) % 3]; // Wrap around

                if (doLinesIntersect(r1, r2, t1, t2)) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * Calculates the coordinates of the 4 robot corners based on current pose and dimensions.
     */
    private Pose2d[] getRobotCorners(Pose pose) {
        Pose2d[] corners = new Pose2d[4];
        double heading = pose.getHeading();
        double x = pose.getX();
        double y = pose.getY();

        // Half dimensions
        double dx = ROBOT_LENGTH / 2.0;
        double dy = ROBOT_WIDTH / 2.0;

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Calculate 4 corners:
        // 0: Front Left (+x, +y)
        // 1: Front Right (+x, -y)
        // 2: Back Left (-x, +y)
        // 3: Back Right (-x, -y)

        corners[0] = new Pose2d(x + (dx * cos - dy * sin), y + (dx * sin + dy * cos), 0);
        corners[1] = new Pose2d(x + (dx * cos - (-dy) * sin), y + (dx * sin + (-dy) * cos), 0);
        corners[2] = new Pose2d(x + (-dx * cos - dy * sin), y + (-dx * sin + dy * cos), 0);
        corners[3] = new Pose2d(x + (-dx * cos - (-dy) * sin), y + (-dx * sin + (-dy) * cos), 0);

        return corners;
    }

    /**
     * Math Helper: Checks if point P is inside Triangle ABC
     */
    private boolean isPointInTriangle(Pose2d p, Pose2d a, Pose2d b, Pose2d c) {
        double w1 = (a.getX() * (c.getY() - a.getY()) + (p.getY() - a.getY()) * (c.getX() - a.getX()) - p.getX() * (c.getY() - a.getY())) /
                ((b.getY() - a.getY()) * (c.getX() - a.getX()) - (b.getX() - a.getX()) * (c.getY() - a.getY()));

        double w2 = (p.getY() - a.getY() - w1 * (b.getY() - a.getY())) / (c.getY() - a.getY());

        return (w1 >= 0.0) && (w2 >= 0.0) && ((w1 + w2) <= 1.0);
    }

    /**
     * Checks if Line Segment (p1, q1) intersects Line Segment (p2, q2)
     */
    private boolean doLinesIntersect(Pose2d p1, Pose2d q1, Pose2d p2, Pose2d q2) {
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special Cases (collinear)
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;
    }

    /**
     * Helper to find orientation of ordered triplet (p, q, r).
     * 0 = collinear, 1 = clockwise, 2 = counterclockwise
     */
    private int orientation(Pose2d p, Pose2d q, Pose2d r) {
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) -
                (q.getX() - p.getX()) * (r.getY() - q.getY());
        if (val == 0) return 0;
        return (val > 0) ? 1 : 2;
    }

    /**
     * Helper to check if point q lies on segment pr
     */
    private boolean onSegment(Pose2d p, Pose2d q, Pose2d r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }



}