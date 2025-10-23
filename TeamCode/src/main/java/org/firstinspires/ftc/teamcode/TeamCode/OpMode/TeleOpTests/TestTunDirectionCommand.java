package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Commands.setTunDirectionCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;

@TeleOp(name = "test Tun Direction Command", group = "TeleOp Tests")
public class TestTunDirectionCommand extends CommandOpMode {
    GamepadEx gamepad;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        super.reset();

        Tun tun = new Tun(hardwareMap);
        PivotTun pivotTun = new PivotTun(hardwareMap);

        register(tun, pivotTun);
        tun.init();
        pivotTun.init();

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 50, Tun.tunState.FORWARD)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 100, Tun.tunState.REVERSE)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 0, Tun.tunState.IDLE)
        );

        super.run();
    }

    @Override
    public void run() {
        telemetry.addData("Motor Power", Tun.getTunPower());
        telemetry.addData("Band Power", Tun.getBandPower());
        telemetry.addData("Current state", Tun.getCurrentTunState());
        telemetry.addData("pivot power", PivotTun.getPIVOT_POWER());
        telemetry.addData("target position", PivotTun.getTARGET_POSITION());
        telemetry.addData("current position" , PivotTun.getCurrentPosition());
        telemetry.addData("tolerance", PivotTun.getTolerance());
        telemetry.update();

        super.run();
    }
}
