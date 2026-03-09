package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.ColorSensor;

@Config
public class CheckLoadCommand extends CommandBase {
    private final ColorSensor colorSensor;

    // We highly recommend checking your telemetry and tuning this value!
    public static double DISTANCE_THRESHOLD_INCHES = 0.8;

    // Time (in ms) the ball must continuously block the sensor to count as "full"
    public static double check = 250;

    private boolean isDetecting = false;
    private boolean loadFull = false;

    // Timer to track how long the ball has been sitting there
    private final ElapsedTime checkTimer;

    public CheckLoadCommand(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        this.checkTimer = new ElapsedTime();
    }

    @Override
    public void execute() {
        boolean ballDetected = colorSensor.distance(DistanceUnit.INCH) < DISTANCE_THRESHOLD_INCHES;

        if (ballDetected) {
            if (!isDetecting) {
                // 1. The ball JUST arrived! Start the timer.
                checkTimer.reset();
                isDetecting = true;
            } else if (checkTimer.milliseconds() > check) {
                // 2. The ball has been sitting here continuously longer than 'check' ms.
                // The robot is full!
                loadFull = true;
            }
        } else {
            // 3. No ball is here, or the ball just left. Reset the timer and states!
            isDetecting = false;
            loadFull = false;
        }
    }

    @Override
    public boolean isFinished() {
        // Command will only end when the ball has sat there for the required time
        return loadFull;
    }
}