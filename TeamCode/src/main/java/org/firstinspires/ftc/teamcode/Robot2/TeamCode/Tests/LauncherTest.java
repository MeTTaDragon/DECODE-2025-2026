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
    // Edit these numbers in Dashboard to test different speeds
    public static double TARGET_VELOCITY_HIGH = 1500;
    public static double TARGET_VELOCITY_LOW = 900;

    // Helper state
    private boolean isHighSpeed = true;

    @Override
    public void initialize() {
        // Setup Dashboard Telemetry (Crucial for the graph!)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcher = new Launcher(hardwareMap);
        register(launcher);

        gamepad = new GamepadEx(gamepad1);

        // Constant loop to send data to the graph
        launcher.setDefaultCommand(new RunCommand(() -> {
            double currentVel = launcher.getVelocity();
            double target = isHighSpeed ? TARGET_VELOCITY_HIGH : TARGET_VELOCITY_LOW;

            // If the launcher is stopped (target 0), we don't graph errors
            // But if it's running, we graph the setpoint
            if(launcher.getVelocity() > 10 || target > 0) {
                telemetry.addData("Target Velocity", target);
                telemetry.addData("Actual Velocity", currentVel);
                telemetry.addData("Error", target - currentVel);
            }

            telemetry.addData("Active F", Launcher.F);
            telemetry.addData("Active P", Launcher.P);
            telemetry.update();
        }, launcher));

        // TRIANGLE: Toggle between High and Low speed
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    isHighSpeed = !isHighSpeed;
                    double newTarget = isHighSpeed ? TARGET_VELOCITY_HIGH : TARGET_VELOCITY_LOW;
                    launcher.setTargetVelocity(newTarget);
                })
        );

        // CROSS: Stop the launcher
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    launcher.stop();
                    isHighSpeed = false; // Reset toggle
                })
        );
    }
}