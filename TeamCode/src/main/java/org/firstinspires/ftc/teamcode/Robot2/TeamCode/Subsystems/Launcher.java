
package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;



import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;


@Config
public class Launcher extends SubsystemBase {
    // "Master" = motorDreapta (The one with the Encoder Cable plugged in)
    private final DcMotorEx masterMotor;

    // "Follower" = motorStanga (Encoder ignored, just follows power)
    private final DcMotorEx followerMotor;

    Follower follower;
    Telemetry telemetry;
    private final Servo hoodServo;
    private final Servo stopper;
    public static double farHoodPose = 0;
    public static double closeHoodPose = 0.45;
    public static double middle_Y = 60;

    public static double stopperClose = 0.25;
    public static double stopperOpen = 0.55;


    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.
    public static double F = 0.00036239;
    public static double P = 0.01;
    public static double D = 0;
    //vel far zone: 1940
    //vel close middle: 1200
    //vel next to goal:
    public static double targetVelocity = 0.0;

    private PIDFController launcherController;

    Pose goalPose;


    public enum LauncherState{
        IDLE,
        SHOOTING
    }

    public enum StopperState{
        MANUAL,
        AUTO
    }



    LauncherState currentLauncherState = LauncherState.IDLE;
    StopperState currentStopperState = StopperState.AUTO;


    public StopperState getCurrentStopperState() {
        return currentStopperState;
    }

    public void setCurrentStopperState(StopperState currentStopperState) {
        this.currentStopperState = currentStopperState;
    }

    public LauncherState getCurrentLauncherState() {
        return currentLauncherState;
    }

    public void setCurrentLauncherState(LauncherState currentLauncherState) {
        this.currentLauncherState = currentLauncherState;
    }

    public Launcher(HardwareMap hwMap, Follower flwr, Telemetry telemetry) {
        // 1. Hardware Mapping - HERE is where we define the Master
        masterMotor = hwMap.get(DcMotorEx.class, "motorDreapta"); // MUST have encoder cable
        followerMotor = hwMap.get(DcMotorEx.class, "motorStanga");// Encoder optional/ignored
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        stopper = hwMap.get(Servo.class, "stopper");
        follower = flwr;
        this.telemetry = telemetry;



        // 3. Set to RUN_WITHOUT_ENCODER
        // This is CRITICAL. It tells the internal REV hub "Don't use your built-in PID,
        // let me handle the math myself in the periodic() function."
        masterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 4. Float behavior for smoother deceleration
        masterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 5. Direction Setup
        // Check this physically! Usually, flywheels spin opposite ways to shoot forward.
        // If the robot shoots backward, remove this REVERSE or move it to masterMotor.
        masterMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        //stopper.setDirection(Servo.Direction.REVERSE);

        launcherController = new PIDFController(P, 0, D, F);

        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
    }


    void updateLauncherState(){
        switch (currentLauncherState){
            case IDLE:
                setTargetVelocity(0);
                masterMotor.setVelocity(0);
                followerMotor.setVelocity(0);
                break;

            case SHOOTING:
                launcherController.setPIDF(P, 0, D, F);

                double currentVel = getVelocity();

                // 2. CALCULATE Error
                double error = targetVelocity - currentVel;

                // 3. CALCULATE Power (PF Controller)
                // Feedforward (F): Base power to maintain target
                // Proportional (P): Correction power based on error
                double power = launcherController.calculate(0, error);

                // 4. CLAMP power to safe range (-1.0 to 1.0)
                power = Math.max(-1.0, Math.min(1.0, power));

                // 5. APPLY the SAME calculated power to BOTH motors
                // This ensures they stay synced, driven by motorDreapta's encoder data.
                masterMotor.setPower(power);
                followerMotor.setPower(power);
                break;
        }
    }

    void updateStopperState(){
        switch (currentStopperState){
            case MANUAL:

                break;
            case AUTO:
                if(getVelocity() > getTargetVelocity() - 30){
                    setStopperPose(stopperOpen);
                } else {
                    setStopperPose(stopperClose);
                }
                break;
        }
    }



    public void init(){
        setCurrentLauncherState(LauncherState.IDLE);
        setCurrentStopperState(StopperState.AUTO);
    }

    /**
     * Sets the target velocity for the flywheel in Ticks Per Second.
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }
    public double getTargetVelocity(){
        return targetVelocity;
    }
    /**
     * Stops the flywheel.
     */
    public void stop() {
        this.targetVelocity = 0;
    }

    /**
     * Returns the current velocity strictly from the MASTER motor (motorDreapta).
     */
    public double getVelocity() {
        return masterMotor.getVelocity();
    }
    public void setHoodPose(double pos) {
        hoodServo.setPosition(pos);
    }

    public void setStopperPose(double pos) {
        stopper.setPosition(pos);
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(follower.getPose().getX() - goalPose.getX(),2) + Math.pow(follower.getPose().getY() - goalPose.getY(),2));
    }

    /**
     * The heartbeat of the subsystem. This runs constantly to update motor power.
     */
    @Override
    public void periodic() {
        updateStopperState();
        updateLauncherState();

        if (follower.getPose().getY() < middle_Y) {
            setHoodPose(farHoodPose);
        } else {
            setHoodPose(closeHoodPose);
        }

        //completeaza cu functia de distanta
        targetVelocity = getDistance() * 10;
    }
}
