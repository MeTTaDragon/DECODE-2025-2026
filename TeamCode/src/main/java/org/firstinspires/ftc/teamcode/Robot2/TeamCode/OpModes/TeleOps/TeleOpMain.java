
package org.firstinspires.ftc.teamcode.Robot2.TeamCode.OpModes.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
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
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.Timing;

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

@TeleOp(name = "TeleOp Main Robo2", group = "Main")
public class TeleOpMain extends CommandOpMode {
    private static double veltarget =1940;
    GamepadEx controller;

    Follower follower;
    Servo led;

    Turret turret;
    Launcher launcher;
    Intake intake;
    LimelightSubsystem limelight;

    List<LynxModule> allHubs;
    ElapsedTime timer;
    Gamepad.RumbleEffect customRumbleEffect;


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

        led = hardwareMap.get(Servo.class, "led");
        turret = new Turret(hardwareMap, follower);
        launcher = new Launcher(hardwareMap, follower);
        intake = new Intake(hardwareMap);
        limelight = new LimelightSubsystem(hardwareMap);

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
                new InstantCommand(() -> veltarget-=25)
        );
        //creste velocity
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> veltarget+=25)
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

        leftTrigger.whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING);
                            turret.setTurretState(Turret.TurretState.FULL_PINPOINT);
                            launcher.setStopperPose(stopperOpen);
                        }),
                        new WaitCommand(2000),
                        new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE))
                )
        );
        leftTrigger.whenInactive(
                new InstantCommand(() -> {
                    launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE);
                    turret.setTurretState(Turret.TurretState.IDLE);
                    launcher.setStopperPose(stopperClose);
                    intake.setIntakeState(Intake.IntakeState.IDLE);
                })
        );
        //intake trage
        rightTrigger.whenActive(
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

//        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(
//                new InstantCommand(() -> {
//                    turret.setTurretState(Turret.TurretState.FULL_PINPOINT);
//                    launcher.setStopperPose(stopperOpen);
//                    launcher.Manual_shooting = true;
//                    launcher.setManualVelocity(veltarget);
//                })
//        );
//        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(
//                new InstantCommand(() -> {
//                    turret.setTurretState(Turret.TurretState.IDLE);
//                    launcher.setStopperPose(stopperClose);
//                    launcher.Manual_shooting = false;
//                    launcher.stop();
//                })
//        );

        //reset position odometrie
        controller.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand(() ->  {
                    follower.setPose(new Pose(72, 7.5, Math.toRadians(90)));
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


        register(turret, launcher, intake);
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
        if(alliance == Alliance.RED) {
            led.setPosition(0.27);
        } else{
            led.setPosition(0.1);
        }



        telemetry.addData("Current velocity", launcher.getVelocity());
        telemetry.addData("Target velocity", launcher.getTargetVelocity());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
        telemetry.addData("Set Point", Math.toDegrees(turret.getSetPoint()));
        telemetry.addData("distance", launcher.getDistance());
        telemetry.addData("alliance", alliance);
        telemetry.addData("goalPose", turret.goalPose);
        telemetry.addData("Manual vel", veltarget);
        telemetry.addData("Loop Time", timer.milliseconds());

        timer.reset();

        super.run();
        telemetry.update();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
