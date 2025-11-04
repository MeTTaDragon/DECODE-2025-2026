package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.LimelightSubsystem;

@TeleOp(name = "Test Limelight", group = "TeleOp Tests")
public class TestLimelight extends CommandOpMode {

    public GamepadEx gamepad;

    @Override
    public void initialize() {
        LimelightSubsystem limelight = new LimelightSubsystem(hardwareMap);

        super.reset();

        register(limelight);

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> Globals.team_color = Globals.TEAM.RED)
        );
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> Globals.team_color = Globals.TEAM.BLUE)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> limelight.setMode(LimelightSubsystem.LimelightMode.READ_PATTERN))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> limelight.setMode(LimelightSubsystem.LimelightMode.TRACK_ARTIFACT))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> limelight.setMode(LimelightSubsystem.LimelightMode.BASKET))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> limelight.setMode(LimelightSubsystem.LimelightMode.PAUSE))
        );

        limelight.init();
        super.run();
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("tx", LimelightSubsystem.getTx());
        telemetry.addData("ty", LimelightSubsystem.getTy());
        telemetry.addData("ta", LimelightSubsystem.getTa());
        telemetry.addData("robotCoordsX", LimelightSubsystem.getRobotCoordsX());
        telemetry.addData("robotCoordsY", LimelightSubsystem.getRobotCoordsY());
        telemetry.addData("robotCoordsZ", LimelightSubsystem.getRobotCoordsZ());
        telemetry.addData("id", LimelightSubsystem.getId());
        telemetry.update();
    }
}
