package org.firstinspires.ftc.teamcode.Robot1.TeamCode.OpMode.TeleOpTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Subsystems.Tun;

@Config
@TeleOp(name = "Test Tun", group = "TeleOp Tests")
public class TestTun extends CommandOpMode {
    private Tun tun;

    public static Tun.tunState testState = Tun.tunState.IDLE;
    public GamepadEx gamepad;
    public void initialize() {
        tun = new Tun(hardwareMap);

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
                new InstantCommand(() -> tun.setTunState(Tun.tunState.IDLE))
        );

        Trigger leftTrigger = new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2);
        leftTrigger.whenActive(() -> tun.setBandPower(0.3));

        tun.init();
        super.run();
    }

    public void run(){
        super.run();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tun.setTunState(testState);

        telemetry.addData("Motor Power", tun.getTunPower());
        telemetry.addData("Current state", tun.getCurrentTunState());
        telemetry.addData("cross", gamepad1.crossWasPressed());
        telemetry.update();
    }
}

