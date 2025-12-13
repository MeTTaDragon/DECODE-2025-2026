package org.firstinspires.ftc.teamcode.Prototype.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Prototype.Subsystems.Turret;


@TeleOp(name = "Turret PS4 TeleOp", group = "Test")
public class TurretTesting extends CommandOpMode {

    Turret turret;
    GamepadEx gamepad;

    public static final double MAX_TURRET_POWER = 0.9;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        turret = new Turret(hardwareMap);
        super.reset();
        register(turret);

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
               new InstantCommand(() ->
                       turret.setPower(MAX_TURRET_POWER),
                       turret
               )
        );
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() ->
                        turret.setPower(0.6),
                        turret
                )
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() ->
                        turret.setPower(0),
                        turret
                )
        );

    }

    @Override
    public void run() {
        super.run();
    }
}