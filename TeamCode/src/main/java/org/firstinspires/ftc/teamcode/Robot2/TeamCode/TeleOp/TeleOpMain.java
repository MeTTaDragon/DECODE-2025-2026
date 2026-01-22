package org.firstinspires.ftc.teamcode.Robot2.TeamCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.Robot1.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

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
       // follower.setStartingPose(Globals.lastAutoPose); cod normal
        follower.setStartingPose(new Pose(72, 7.5, Math.toRadians(90))); //pozitie setata pentru testari
        alliance = Alliance.BLUE;


        super.reset();
        turret = new Turret(hardwareMap, follower, telemetry);
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.startTeleopDrive(true);

        register(turret, launcher, intake);

        controller.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.REVERSE))
        );

        controller.getGamepadButton(GamepadKeys.Button.CROSS).whenReleased(
                new InstantCommand(() -> intake.setIntakeState(Intake.IntakeState.IDLE))
        );

        controller.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> launcher.setPower(0.6))
        );

        controller.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenReleased(
                new InstantCommand(() -> launcher.setPower(0.0))
        );

        controller.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> launcher.setPower(0.3))
        );

        controller.getGamepadButton(GamepadKeys.Button.SQUARE).whenReleased(
                new InstantCommand(() -> launcher.setPower(0.0))
        );



        super.run();

    }


    public void run() {
        super.run();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
        telemetry.setMsTransmissionInterval(250);

        telemetry.addData("Current state", turret.getCurrentTurretState());
        telemetry.addData("Turret Power", turret.getCurrentPower());
        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
        telemetry.addData("Set Point", Math.toDegrees(turret.getSetPoint()));
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();


        turret.setTurretState(Turret.TurretState.FULL_PINPOINT);
    }
}
