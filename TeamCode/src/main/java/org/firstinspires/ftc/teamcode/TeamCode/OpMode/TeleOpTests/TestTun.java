package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;

@TeleOp(name = "Test Tun")
public class TestTun extends CommandOpMode {

    public GamepadEx gamepad;
    public void initialize() {
        Tun tun = new Tun(hardwareMap);
        
        super.reset();

        register(tun);

        gamepad = new GamepadEx(gamepad1);
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> tun.setTunState(Tun.tunState.FORWARD))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> tun.setTunState(Tun.tunState.REVERSE))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> tun.setTunState(Tun.tunState.STOP))
        );


        tun.init();
    }

    public void run(){
        telemetry.addData("Motor Power", Tun.getTunPower());
        telemetry.addData("Band Power", Tun.getBandPower());
        telemetry.addData("Current state", Tun.getCurrentTunState());
        telemetry.update();
    }
}
