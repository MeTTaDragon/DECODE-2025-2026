package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Commands.RobotCentricDriveCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Commands.setTunDirectionCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;

@TeleOp(name = "TeleOp Structure", group = "TeleOpStructures")
public class TeleOpStruct extends CommandOpMode {

    GamepadEx chassis;
    GamepadEx cannon;
    Tun tun;
    PivotTun pivotTun;
    Drivetrain drive;

    public static Tun.tunState testState = Tun.tunState.IDLE;
    public static int TARGET_POSITION = 0;


    @Override
    public void initialize() {


        chassis = new GamepadEx(gamepad1);
        cannon = new GamepadEx(gamepad2);

        super.reset();

        tun = new Tun(hardwareMap);
        pivotTun = new PivotTun(hardwareMap);
        drive = new Drivetrain(hardwareMap);

        register(tun, pivotTun, drive);
        tun.init();
        pivotTun.init();
        drive.init();

        drive.setDefaultCommand(new RobotCentricDriveCommand(drive, chassis));


        cannon.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, Tun.tunState.FORWARD)
        );
        cannon.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun,  Tun.tunState.REVERSE)
        );
        cannon.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, Tun.tunState.IDLE)
        );
        cannon.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition(1500))
        );
        cannon.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition(0))
        );
        cannon.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition(-1500))
        );

        super.run();
    }

    @Override
    public void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tun.setTunState(testState);
        pivotTun.setPivotPosition(TARGET_POSITION);


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
        telemetry.update();

        super.run();
    }
}
