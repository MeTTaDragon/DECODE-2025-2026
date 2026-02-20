package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public static boolean isChoking;
    private long resetStartTime = 0;
    private boolean isResetting = false;
    private static TurretState stateBeforeReset;


    double targetHeading;

    // PID Coefficients
    // Note: Since we are using Radians, the error is small (e.g., 0.5 rads).
    // You might need a higher P than 0.35 if it's sluggish.
    // Try P = 0.8 or higher if it doesn't move fast enough.
    public static double P = 0.09, I = 0, D = 0.001, F = 0.8;
    public static double ll_P = 0.09, ll_I = 0, ll_D = 0, ll_F = 0.8;
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
        motorTureta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretController = new PIDFController(P, I, D, F);

        // Pedro uses its own Pose class, be careful not to mix up Point/Pose classes
        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
        follower = flwr;

        targetGoalPose = new Pose2d(this.goalPose.getX(), this.goalPose.getY(), 0);

        turretController.setTolerance(0);
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


        switch (currentTurretState){
            case IDLE:
                turretController.setPIDF(0.8, 0, 0.003, 0);
                lltx = 0;
                llty = 0;
                llta = 0;
                power = turretController.calculate(getTurretHeading(), 0);
                motorTureta.setPower(power);
                break;

            case FULL_LIMELIGHT:
                if(lltx < 5){
                    turretController.setPIDF(0,0,0,1.5);
                }
                else{
                    turretController.setPIDF(ll_P, ll_I, ll_D, ll_F);
                }

                turretController.setSetPoint(Math.toRadians(-lltx));

                power = turretController.calculate(0);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                }
                else{
                    motorTureta.setPower(power);
                }
                break;

            case FULL_PINPOINT:
                turretController.setPIDF(P, I, D, F);
                // 1. Get Robot Position & Heading from Pedro
                Pose robotPose = follower.getPose();

                // 2. Calculate the "Field Heading" (Angle from Robot to Goal globally)
                double targetFieldHeading = Math.atan2(targetGoalPose.getY() - robotPose.getY(),
                        targetGoalPose.getX() - robotPose.getX());

                // 3. Calculate "Target Local Heading"
                // This is where the turret needs to be relative to the robot chassis.
                // Formula: Field_Angle - Robot_Body_Angle
                targetHeading = targetFieldHeading - robotPose.getHeading();

                // 4. Get "Current Local Heading" from Encoder
                double currentLocalHeading = getTurretHeading(); // returns Radians

                // 5. Calculate the Error (Shortest Path)
                // This helper function handles the -180 to 180 wrap automatically.
                // If the error is 350 degrees, it converts it to -10 degrees.
                double error = angleWrap(targetHeading - currentLocalHeading);



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
            isChoking = true;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
            isChoking = true;
        }
        return angle;
    }
    public double getTargetHeading() {
        return targetHeading;
    }


    @Override
    public void periodic() {
        robotAngle = follower.getPose().getHeading();


        if (Math.abs(Math.toDegrees(getTurretHeading())) > 170 && !isResetting) {
            stateBeforeReset = getCurrentTurretState();
            setTurretState(TurretState.IDLE);
            //basically wait command 500 ms
            resetStartTime = System.currentTimeMillis();
            isResetting = true;
        }

        // Check if the "wait" is over
        if (isResetting) {
            // Wait for 500ms (0.5 seconds) - adjust as needed for cable safety
            if (System.currentTimeMillis() - resetStartTime > 500) {
                setTurretState(stateBeforeReset);
                isResetting = false; // Reset the flag
            }
        }

        update();
    }
}