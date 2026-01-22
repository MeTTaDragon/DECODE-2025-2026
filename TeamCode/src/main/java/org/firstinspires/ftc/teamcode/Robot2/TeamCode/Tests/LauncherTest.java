package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;

@TeleOp
@Config
public class LauncherTest extends CommandOpMode {
    Launcher launcher;
    GamepadEx gamepad;

    // --- DASHBOARD VARIABLES ---
    // Change these in Dashboard to test overshoot/recovery
    public static double TARGET_VELOCITY_HIGH = 1800;
    public static double TARGET_VELOCITY_LOW = 1000;

    private boolean isHighSpeed = true;

    @Override
    public void initialize() {
        // Setup Dashboard Telemetry to see the graph
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcher = new Launcher(hardwareMap);
        register(launcher);

        gamepad = new GamepadEx(gamepad1);

        // Default Command: Updates Telemetry & maintains speed
        launcher.setDefaultCommand(new RunCommand(() -> {
            // "getVelocity" here calls the Master (motorDreapta) automatically
            double currentVel = launcher.getVelocity();
            double target = isHighSpeed ? TARGET_VELOCITY_HIGH : TARGET_VELOCITY_LOW;

            // Only graph if moving or trying to move
            if(Math.abs(currentVel) > 10 || target > 0) {
                telemetry.addData("Target (SetPoint)", target);
                telemetry.addData("Actual (motorDreapta)", currentVel);
                telemetry.addData("Error", target - currentVel);
            }

            telemetry.addData("Active F", Launcher.F);
            telemetry.addData("Active P", Launcher.P);
            telemetry.update();
        }, launcher));

        // TRIANGLE: Toggle Speed (The "Step Test")
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    isHighSpeed = !isHighSpeed;
                    double newTarget = isHighSpeed ? TARGET_VELOCITY_HIGH : TARGET_VELOCITY_LOW;
                    launcher.setTargetVelocity(newTarget);
                })
        );

        // CROSS: Emergency Stop
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    launcher.stop();
                    isHighSpeed = false;
                })
        );
    }
}