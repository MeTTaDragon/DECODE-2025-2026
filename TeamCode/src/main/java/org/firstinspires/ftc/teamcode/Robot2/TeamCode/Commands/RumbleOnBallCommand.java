package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;

public class RumbleOnBallCommand extends CommandBase {
    private final ColorSensor colorSensor;
    private final Gamepad gamepad;

    // Adjust this value based on how close the ball needs to be
    private final double DISTANCE_THRESHOLD_INCHES = 5.2;

    // State variable to track if the ball is currently in the intake
    private boolean isDetecting = false;

    /**
     * @param colorSensor Your ColorSensor subsystem
     * @param gamepad The gamepad you want to rumble (usually gamepad1 or gamepad2)
     */
    public RumbleOnBallCommand(ColorSensor colorSensor, Gamepad gamepad) {
        this.colorSensor = colorSensor;
        this.gamepad = gamepad;

        // Note: We do NOT use addRequirements(colorSensor) here if we want
        // this command to run constantly in the background without interrupting
        // other commands that might also need to read the color sensor.
    }

    @Override
    public void execute() {
        // Check if the distance is less than our threshold
        boolean ballDetected = colorSensor.distance(DistanceUnit.INCH) < DISTANCE_THRESHOLD_INCHES;

        if (ballDetected && !isDetecting) {
            // The ball just arrived! Tell the gamepad to rumble continuously
            gamepad.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            isDetecting = true;
        } else if (!ballDetected && isDetecting) {
            // The ball left the sensor's range! Stop the rumble immediately
            gamepad.stopRumble();
            isDetecting = false;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}