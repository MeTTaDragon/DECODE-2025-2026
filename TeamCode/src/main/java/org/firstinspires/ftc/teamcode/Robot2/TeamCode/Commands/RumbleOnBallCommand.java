package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;
@Config
public class RumbleOnBallCommand extends CommandBase {
    private final ColorSensor colorSensor;
    private final Gamepad gamepad;
    private final Gamepad gamepad2;

    // We highly recommend checking your telemetry and tuning this value!
    public static double  DISTANCE_THRESHOLD_INCHES = 0.8;

    private boolean isDetecting = false;

    // Add a timer to refresh the rumble before the controller times out
    private final ElapsedTime rumbleTimer;

    public RumbleOnBallCommand(ColorSensor colorSensor, Gamepad gamepad, Gamepad gamepad2) {
        this.colorSensor = colorSensor;
        this.gamepad = gamepad;
        this.gamepad2 = gamepad2;
        this.rumbleTimer = new ElapsedTime();
    }

    @Override
    public void execute() {
        boolean ballDetected = colorSensor.distance(DistanceUnit.INCH) < DISTANCE_THRESHOLD_INCHES;

        if (ballDetected) {
            // If the ball just arrived, OR if 500ms have passed since the last rumble command
            if (!isDetecting || rumbleTimer.milliseconds() > 100) {
                gamepad.rumble(0.5, 0.5, 100); // Send a 500ms rumble
                gamepad2.rumble(0.5, 0.5, 100); // Send a 500ms rumble
                rumbleTimer.reset();           // Reset the timer
                isDetecting = true;
            }
        } else {
            // Ball left the intake
            if (isDetecting) {
                gamepad.stopRumble();
                gamepad2.stopRumble();
                isDetecting = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}