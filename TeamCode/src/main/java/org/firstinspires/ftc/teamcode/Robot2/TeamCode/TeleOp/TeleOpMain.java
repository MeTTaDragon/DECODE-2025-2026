package org.firstinspires.ftc.teamcode.Robot2.TeamCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperClose;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher.stopperOpen;

@TeleOp(name = "TeleOp Main Robo2", group = "Main")
public class TeleOpMain extends CommandOpMode {

    GamepadEx controller;

    Follower follower;

    Turret turret;
    Launcher launcher;
    Intake intake;

    @Override
    public void initialize() {
        controller = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(lastAutoPose);

        super.reset();
        turret = new Turret(hardwareMap, follower, telemetry);
        launcher = new Launcher(hardwareMap, follower, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.startTeleopDrive(true);

        register(turret, launcher, intake);

        turret.setTurretState(Turret.TurretState.IDLE);
        launcher.init();

        controller.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.IDLE))
        );

        controller.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.FULL_PINPOINT))
        );

        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    launcher.setCurrentStopperState(Launcher.StopperState.MANUAL);
                    launcher.setStopperPose(stopperOpen);
                })
        );
        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    launcher.setCurrentStopperState(Launcher.StopperState.MANUAL);
                    launcher.setStopperPose(stopperClose);
                })
        );
        controller.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> launcher.setCurrentStopperState(Launcher.StopperState.AUTO))
        );



        Trigger rightTrigger = new Trigger(() -> gamepad1.right_trigger > 0.1);
        Trigger leftTrigger = new Trigger(() -> gamepad1.left_trigger > 0.1);

        leftTrigger.whenActive(
                new InstantCommand(() -> launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING))
        );
        leftTrigger.whenInactive(
                new InstantCommand(() -> launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE))
        );

        rightTrigger.whenActive(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE))
        );
        rightTrigger.whenInactive(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.IDLE))
        );

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.FORWARD))
        );
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.IDLE))
        );

        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    launcher.setCurrentStopperState(Launcher.StopperState.AUTO);
                    turret.setTurretState(Turret.TurretState.IDLE);
                })
        );

        controller.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> launcher.setCurrentLauncherState(Launcher.LauncherState.SHOOTING))
        );
        controller.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(
                new InstantCommand(() -> launcher.setCurrentLauncherState(Launcher.LauncherState.IDLE))
        );

        telemetry.setMsTransmissionInterval(250);

        super.run();
    }


    public void run() {
        super.run();
        if(alliance == Alliance.RED){
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        } else{
            follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        }

        follower.update();



        telemetry.addData("Current velocity", launcher.getVelocity());
        telemetry.addData("Target velocity", launcher.getTargetVelocity());
        telemetry.addData("launcher state", launcher.getCurrentLauncherState());
        telemetry.addData("stopper state", launcher.getCurrentStopperState());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Current state", turret.getCurrentTurretState());
        telemetry.addData("Turret Power", turret.getCurrentPower());
        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
        telemetry.addData("Set Point", Math.toDegrees(turret.getSetPoint()));
        telemetry.addData("distance", launcher.getDistance());
        telemetry.update();
    }
}
