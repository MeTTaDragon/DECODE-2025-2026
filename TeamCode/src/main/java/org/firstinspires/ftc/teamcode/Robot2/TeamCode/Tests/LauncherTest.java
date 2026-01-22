package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot1.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;

@TeleOp
public class LauncherTest extends CommandOpMode {
    Launcher launcher;
    GamepadEx gamepad;
    Follower follower;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.reset();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 7.5, Math.toRadians(90))); //pozitie setata pentru testari

        launcher = new Launcher(hardwareMap, follower, telemetry);

        register(launcher);

        gamepad = new GamepadEx(gamepad1);


        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> launcher.setPower(0.5))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> launcher.setPower(1))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> launcher.stop())
        );
    }
}
