package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

@Config
public class Turret extends SubsystemBase {

    private final DcMotorEx motorTureta;
    private final PIDFController turretController;
    private final Follower follower;

    public Pose goalPose;
    public Pose2d targetGoalPose;

    // Variables for logic
    double robotAngle;
    double power;

    // PID Coefficients
    // Note: Since we are using Radians, the error is small (e.g., 0.5 rads).
    // You might need a higher P than 0.35 if it's sluggish.
    // Try P = 0.8 or higher if it doesn't move fast enough.
    public static double P = 0.8, I = 0, D = 0.03, F = 0;

    // Hardware Constants
    double gearRatio = 5.75;
    double TicksPerRev = 145.1; // Motor internal PPR

    public enum TurretState {
        IDLE,
        FULL_LIMELIGHT,
        FULL_PINPOINT,
        MIXED,
    }
    private static TurretState currentTurretState = TurretState.IDLE;

    public Turret(HardwareMap hwMap, Follower flwr) {
        motorTureta = hwMap.get(DcMotorEx.class, "motorTureta");

        // CHECK THIS: Ensure Positive Power = Counter-Clockwise rotation
        motorTureta.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTureta.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretController = new PIDFController(P, I, D, F);

        // Pedro uses its own Pose class, be careful not to mix up Point/Pose classes
        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
        follower = flwr;

        targetGoalPose = new Pose2d(this.goalPose.getX(), this.goalPose.getY(), 0);
    }

    public void setTurretState(TurretState state) {
        currentTurretState = state;
    }

    public TurretState getCurrentTurretState(){ return currentTurretState; }
    public double getPower(){ return motorTureta.getPower(); }
    public double getSetPoint(){ return turretController.getSetPoint(); }
    public double getCurrentPower(){
        return power;}

    void update() {
        // Update PID coefficients from Dashboard
        turretController.setPIDF(P, I, D, F);

        switch (currentTurretState){
            case IDLE:
                power = turretController.calculate(getTurretHeading(), 0);
                motorTureta.setPower(power);
                break;

            case FULL_LIMELIGHT:
                turretController.setSetPoint(0);

                power = turretController.calculate(lltx);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                }
                else{
                    motorTureta.setPower(-power);
                }
                break;

            case FULL_PINPOINT:
                // 1. Get Robot Position & Heading from Pedro
                Pose robotPose = follower.getPose();

                // 2. Calculate the "Field Heading" (Angle from Robot to Goal globally)
                double targetFieldHeading = Math.atan2(targetGoalPose.getY() - robotPose.getY(),
                        targetGoalPose.getX() - robotPose.getX());

                // 3. Calculate "Target Local Heading"
                // This is where the turret needs to be relative to the robot chassis.
                // Formula: Field_Angle - Robot_Body_Angle
                double targetLocalHeading = targetFieldHeading - robotPose.getHeading();

                // 4. Get "Current Local Heading" from Encoder
                double currentLocalHeading = getTurretHeading(); // returns Radians

                // 5. Calculate the Error (Shortest Path)
                // This helper function handles the -180 to 180 wrap automatically.
                // If the error is 350 degrees, it converts it to -10 degrees.
                double error = angleWrap(targetLocalHeading - currentLocalHeading);

                // 6. PID Calculation
                // We calculate power to drive 'error' to 0.
                // Note: PIDFController.calculate(measured, setpoint)
                // We pass 0 as measured and error as setpoint (or vice versa depending on sign)
                // Effectively: P * error
                power = turretController.calculate(0, error);

                motorTureta.setPower(power);
                break;

            case MIXED:
                break;
        }
    }

    /**
     * Converts motor ticks to Local Turret Radians (0 is front of robot)
     */
    public double getTurretHeading() {
        double ticks = motorTureta.getCurrentPosition();
        // Formula: (Ticks / Total_Ticks_Per_Rev) * 2PI
        return (ticks / (TicksPerRev * gearRatio)) * 2 * Math.PI;
    }

    /**
     * Normalizes an angle to be between -PI and +PI
     * This ensures the turret takes the shortest path.
     */
    private double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    @Override
    public void periodic() {
        robotAngle = follower.getPose().getHeading();
        update();
    }
}