package org.firstinspires.ftc.teamcode.TeamCode.OpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

public class TeleOpPrezentare extends CommandOpMode {
    GamepadEx gamepad;
    Tun tun;
    PivotTun pivotTun;
    LimelightSubsystem limelight;
    Drivetrain drive;

    public static Tun.tunState testState = Tun.tunState.IDLE;
    public static int TARGET_POSITION = 0;


    @Override
    public void initialize() {


        gamepad = new GamepadEx(gamepad1);

        super.reset();

        limelight = new LimelightSubsystem(hardwareMap);

        tun = new Tun(hardwareMap);
        pivotTun = new PivotTun(hardwareMap);

        register(tun, pivotTun, limelight, drive);
        tun.init();
        pivotTun.init();
        limelight.init();
        drive.init();

        drive.setDefaultCommand(new RobotCentricDriveCommand(drive, gamepad));


        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, limelight, 50, Tun.tunState.FORWARD)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, limelight, 100, Tun.tunState.REVERSE)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, limelight, 0, Tun.tunState.IDLE)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition(1000))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition(0))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> pivotTun.setPivotPosition(-1000))
        );

        super.run();
    }

    @Override
    public void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tun.setTunState(testState);
        pivotTun.setPivotPosition(TARGET_POSITION);


        telemetry.addData("Motor Power", Tun.getTunPower());
        telemetry.addData("Band Power", Tun.getBandPower());
        telemetry.addData("Current state", Tun.getCurrentTunState());
        telemetry.addData("pivot power", PivotTun.getPIVOT_POWER());
        telemetry.addData("target position", PivotTun.getTARGET_POSITION());
        telemetry.addData("current position" , PivotTun.getCurrentPosition());
        telemetry.addData("tolerance", PivotTun.getTolerance());
        telemetry.addData("tun current power dreapta", Tun.getCurrentSpeedDreapta());
        telemetry.addData("tun current power stanga", Tun.getCurrentSpeedStanga());
        telemetry.update();

        super.run();
    }
}
