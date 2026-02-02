
package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.TeleOps;

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
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.closeHoodPose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.farHoodPose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperClose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperOpen;

import java.util.List;
@Config
@TeleOp(name = "TeleOp Main Robo2", group = "Main")
public class TeleOpMain extends CommandOpMode {
    private static double veltarget =1940;
    public static double ledcolor = 0.278;
    GamepadEx controller;


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

    List<LynxModule> allHubs;
    ElapsedTime timer;
    Gamepad.RumbleEffect customRumbleEffect;
    boolean rumbled = false;
    private static double totallooptime=0;
    private static double loops=0;



    @Override
    public void initialize() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        controller = new GamepadEx(gamepad1);

        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 100)
                .build();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(lastAutoPose);

        super.reset();

        ledAlliance = hardwareMap.get(Servo.class, "ledAlliance");
        ledShooter = hardwareMap.get(Servo.class, "ledShooter");
        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap, follower);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //telemetry.setMsTransmissionInterval(250);

        follower.startTeleopDrive(true);



        turret.setTurretState(Turret.TurretState.IDLE);
        limelight.init();
        launcher.init();

        controller.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.IDLE), turret)
        );

        controller.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.FULL_PINPOINT), turret)
        );
        //hood far zone
        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    launcher.setHoodPose(farHoodPose);
                })
        );
        //hood close zone
        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    launcher.setHoodPose(closeHoodPose);
                })
        );
        //scade velocity
        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> launcher.useLimelight = false)
        );
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> launcher.useLimelight = true)
        );
        //open stopper
        controller.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> launcher.setStopperPose(stopperOpen))
        );
        //close stopper
        controller.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    launcher.setStopperPose(stopperClose);
                })
        );



        Trigger rightTrigger = new Trigger(() -> gamepad1.right_trigger > 0.1);
        Trigger leftTrigger = new Trigger(() -> gamepad1.left_trigger > 0.1);

        leftTrigger.whileActiveOnce(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() ->launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING)),
                                new InstantCommand(() ->turret.setTurretState(Turret.TurretState.FULL_PINPOINT)),
                                new InstantCommand(() -> limelight.setMode(LimelightSubsystem.LimelightMode.BASKET))
                        ),
                        new WaitUntilCommand(() -> launcher.isVelocityReached()),
                        new InstantCommand(() -> turret.setTurretState(Turret.TurretState.FULL_LIMELIGHT)),
                        new InstantCommand(() -> launcher.setStopperPose(stopperOpen)),
                        new WaitCommand(500),
                        new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE))
                )
        );
        leftTrigger.whenInactive(
                new InstantCommand(() -> {
                    launcher.stop();
                    turret.setTurretState(Turret.TurretState.IDLE);
                    launcher.setStopperPose(stopperClose);
                    intake.setIntakeState(Intake.IntakeState.IDLE);
                    limelight.setMode(LimelightSubsystem.LimelightMode.PAUSE);
                })
        );
        //intake trage
        rightTrigger.whileActiveOnce(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE), intake)
        );
        rightTrigger.whenInactive(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.IDLE), intake)
        );

        //intake scuipa
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD), intake)
        );
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.IDLE), intake)
        );

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING);
                            limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);
                            turret.setTurretState(Turret.TurretState.FULL_LIMELIGHT);
                        }),
                        new WaitUntilCommand(() -> launcher.isVelocityReached()),
                        new InstantCommand(() -> launcher.setStopperPose(stopperOpen)),
                        new WaitCommand(500),
                        new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE))
                )
        );
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(
                new InstantCommand(() -> {
                    turret.setTurretState(Turret.TurretState.IDLE);
                    launcher.setStopperPose(stopperClose);
                    launcher.stop();
                    limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);
                    intake.setIntakeState(Intake.IntakeState.IDLE);
                })
        );

        //reset position odometrie
        controller.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand(() ->  {
                    follower.setPose(new Pose(72, 7.5, Math.toRadians(90)));
                    rumbled = false;
                    telemetry.addData("Status", "Pose Reset Triggered");
                }
                )
        );
        //setare manuala alianta albastra
        controller.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(() ->  {
                    alliance = Alliance.BLUE;
                    turret.goalPose =  blueGoalPose;
                    turret.targetGoalPose = new Pose2d(turret.goalPose.getX(), turret.goalPose.getY(), 0);
                }
                )
        );
        //setare manuala alianta rosie
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new InstantCommand(() ->  {
                    alliance = Alliance.RED;
                    turret.goalPose =  redGoalPose;
                    turret.targetGoalPose = new Pose2d(turret.goalPose.getX(), turret.goalPose.getY(), 0);
                }
                )
        );


        register(turret, launcher, intake, limelight);
    }
    /**
     * Updates the LED color based on Robot Position and Alliance
     */
    public void updateLEDs() {
        // Get current robot position from Pedro Pathing
        Pose currentPose = follower.getPose();
        Pose2d robotPoint = new Pose2d(currentPose.getX(), currentPose.getY(), 0);

        // Check if robot is inside either triangle
        boolean inFront = isPointInTriangle(robotPoint, frontA, frontB, frontC);
        boolean inBack  = isPointInTriangle(robotPoint, backA, backB, backC);

        if (inFront || inBack) {
            // Force WHITE if inside a launch triangle
            ledShooter.setPosition(0.5);
        }
        else{
            ledShooter.setPosition(1);
        }


        if (alliance == Alliance.RED) {
            ledAlliance.setPosition(0.28);
        } else if (alliance == Alliance.BLUE) {
            ledAlliance.setPosition(0.61);
        } else {
            ledAlliance.setPosition(0);
        }
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


    public void run() {
        timer.reset();

        /*if(alliance == Alliance.RED){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        } else{
            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        }*/
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();


        updateLEDs();

        if((follower.getPose().getX() < 0 || follower.getPose().getX() > 144 || follower.getPose().getY() < 0 || follower.getPose().getY() > 144) && !rumbled){
            rumbled = true;
            gamepad1.runRumbleEffect(customRumbleEffect);
        }


        telemetry.addData("Current velocity", launcher.getVelocity());
        telemetry.addData("Target velocity", launcher.getTargetVelocity());
//        telemetry.addData("Robot X", follower.getPose().getX());
//        telemetry.addData("Robot Y", follower.getPose().getY());
//        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
//        telemetry.addData("turret target heading", Math.toDegrees(turret.getTargetHeading()));
        telemetry.addData("distance", launcher.getDistance());
//        telemetry.addData("alliance", alliance);
        telemetry.addData("limelight mode", limelight.getCurrentMode());
        telemetry.addData("llta", llta);
        telemetry.addData("tx", lltx);
        telemetry.addData("ty", llty);

        telemetry.addData("Loop Time", 1/timer.seconds());
        totallooptime+=1/timer.seconds();
        loops++;
        telemetry.addData("Average Looptime", totallooptime/loops);


        timer.reset();

        super.run();
        telemetry.update();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
