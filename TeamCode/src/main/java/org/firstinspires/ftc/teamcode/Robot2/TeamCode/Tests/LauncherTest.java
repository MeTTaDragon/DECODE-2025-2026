package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor; // Import DcMotor
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
    DcMotor intake; // 1. Define the intake motor
    GamepadEx gamepad;

    // --- DASHBOARD VARIABLES ---
    public static double TARGET_VELOCITY_HIGH = 1940;
    public static double TARGET_VELOCITY_LOW = 1200;

    private boolean isHighSpeed = true;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcher = new Launcher(hardwareMap);
        register(launcher);

        // 2. Initialize the Intake Motor
        // Make sure your config on the phone/hub names this motor "intake"
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad = new GamepadEx(gamepad1);

        launcher.setDefaultCommand(new RunCommand(() -> {
            double currentVel = launcher.getVelocity();
            double target = isHighSpeed ? TARGET_VELOCITY_HIGH : TARGET_VELOCITY_LOW;

            if(Math.abs(currentVel) > 10 || target > 0) {
                telemetry.addData("Target (SetPoint)", target);
                telemetry.addData("Actual (motorDreapta)", currentVel);
                telemetry.addData("Error", target - currentVel);
            }

            telemetry.addData("Active F", Launcher.F);
            telemetry.addData("Active P", Launcher.P);
            // Optional: See intake status
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();
        }, launcher));

        // TRIANGLE: Toggle Speed AND Turn Intake ON
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> {
                    isHighSpeed = !isHighSpeed;
                    double newTarget = isHighSpeed ? TARGET_VELOCITY_HIGH : TARGET_VELOCITY_LOW;

                    // Set Launcher Speed
                    launcher.setTargetVelocity(newTarget);

                    // 3. Turn Intake ON
                    intake.setPower(-1.0);
                })
        );

        // CROSS: Emergency Stop AND Turn Intake OFF
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    // Stop Launcher
                    launcher.stop();
                    isHighSpeed = false;

                    // 4. Turn Intake OFF
                    intake.setPower(0.0);
                })
        );
    }
}