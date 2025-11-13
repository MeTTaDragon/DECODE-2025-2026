package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;

@Config
@TeleOp(name = "Test Pivot", group = "TeleOp Tests")
public class TestPivot extends CommandOpMode {
    //TODO:verifica dupa ce dai push de pe laptopu atlas daca e bine scris
    GamepadEx gamepad;
    PivotTun pivotTun;

    public static int TARGET_POSITION = 0;

    @Override
    public void initialize() {
        pivotTun = new PivotTun(hardwareMap);
        gamepad = new GamepadEx(gamepad1);

        super.reset();

        register(pivotTun);
        pivotTun.init();

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition())
        );

        super.run();
    }

    @Override
    public void run(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pivotTun.setPivotPosition(TARGET_POSITION);

        telemetry.addData("pivot power", PivotTun.getPIVOT_POWER());
        telemetry.addData("target position", PivotTun.getTARGET_POSITION());
        telemetry.addData("current position" , PivotTun.getCurrentPosition());
        telemetry.addData("tolerance", PivotTun.getTolerance());
        telemetry.update();



        super.run();
    }
}
