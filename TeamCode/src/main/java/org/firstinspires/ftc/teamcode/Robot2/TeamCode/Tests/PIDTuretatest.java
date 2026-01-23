package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

@Config
@TeleOp(name = "PID tureta test")
public class PIDTuretatest extends CommandOpMode {
    Turret tureta;
    GamepadEx gamepad;

    public static double testPoint = 0.0;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.reset();
        tureta = new Turret(hardwareMap, null, telemetry);
        register(tureta);

        gamepad = new GamepadEx(gamepad1);



        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    //tureta.setTestPoint(3.14);
                    tureta.setTurretState(Turret.TurretState.TEST);
                })
        );

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                   // tureta.setTestPoint(0);
                    tureta.setTurretState(Turret.TurretState.TEST);
                })
        );

        super.run();
    }

    @Override
    public void run() {
        telemetry.setMsTransmissionInterval(250);

        telemetry.update();

        super.run();
    }
}
