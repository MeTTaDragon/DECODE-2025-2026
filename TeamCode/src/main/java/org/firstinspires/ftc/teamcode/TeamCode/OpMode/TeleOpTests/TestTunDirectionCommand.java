package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

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
                new setTunDirectionCommand(tun, pivotTun, 50, Tun.tunState.FORWARD)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 100, Tun.tunState.REVERSE)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 0, Tun.tunState.IDLE)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
               new InstantCommand(() -> PivotTun.motorPivot.setPower(0))

        );
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> PivotTun.failsafe=true)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> pivotTun.findMaxPosition())

        );

        super.run();
    }

    @Override
    public void run() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tun.setTunState(testState);
        pivotTun.setPivotPosition(TARGET_POSITION);


        telemetry.addData("Motor Stanga Power", Tun.motorStanga.getPower());
        telemetry.addData("Motor Dreapta Power", Tun.motorDreapta.getPower());
        telemetry.addData("Band Power", Tun.getBandPower());
        telemetry.addData("Current state", Tun.getCurrentTunState());
        telemetry.addData("pivot power", PivotTun.getPIVOT_POWER());
        telemetry.addData("target position", PivotTun.getTARGET_POSITION());
        telemetry.addData("current position" , PivotTun.getCurrentPosition());
        telemetry.addData("tolerance", PivotTun.getTolerance());
        telemetry.addData("tun current power dreapta", Tun.getCurrentSpeedDreapta());
        telemetry.addData("tun current power stanga", Tun.getCurrentSpeedStanga());
        telemetry.addData("---failsafe:", PivotTun.failsafe);
        telemetry.addData("---Switch pressed", PivotTun.getSwitchState());
        telemetry.update();

        super.run();
    }
}
