package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Commands.FindMaxPosPivotCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Commands.RobotCentricDriveCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Commands.setTunDirectionCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;
@Config
@TeleOp(name = "test Tun Direction Command", group = "TeleOp Tests")
public class TestTunDirectionCommand extends CommandOpMode {
    GamepadEx gamepad;
    Tun tun;
    PivotTun pivotTun;

    public static Tun.tunState testState = Tun.tunState.IDLE;
    public static int TARGET_POSITION = 0;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);

        super.reset();

        tun = new Tun(hardwareMap);
        pivotTun = new PivotTun(hardwareMap);

        register(tun, pivotTun);
        tun.init();
        pivotTun.init();



        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 1800)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, -1800)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 0)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
               new InstantCommand(() -> pivotTun.motorPivot.setPower(0))

        );

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new FindMaxPosPivotCommand(pivotTun).interruptOn(
                        () -> { return gamepad1.square; }
                )
        );

        super.run();
    }

    @Override
    public void run() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        tun.setTunState(testState);
//        pivotTun.setPivotPosition(TARGET_POSITION);


        telemetry.addData("Motor Stanga Power", tun.motorStanga.getPower());
        telemetry.addData("Motor Dreapta Power", tun.motorDreapta.getPower());
        telemetry.addData("Band Power", tun.getBandPower());
        telemetry.addData("Current state", tun.getCurrentTunState());
        telemetry.addData("pivot power", pivotTun.getPIVOT_POWER());
        telemetry.addData("target position", pivotTun.getTARGET_POSITION());
        telemetry.addData("current position" , pivotTun.getCurrentPosition());
        telemetry.addData("tolerance", pivotTun.getTolerance());
        telemetry.addData("tun current power dreapta", tun.getCurrentSpeedDreapta());
        telemetry.addData("tun current power stanga", tun.getCurrentSpeedStanga());
        telemetry.addData("---Switch pressed", pivotTun.getSwitchState());

        telemetry.addData("cross", gamepad1.cross);
        telemetry.update();

        super.run();
    }
}
