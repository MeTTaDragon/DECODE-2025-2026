
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

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;
import static java.lang.Math.abs;


@Config
public class Launcher extends SubsystemBase {
    // "Master" = motorDreapta (The one with the Encoder Cable plugged in)
    private final DcMotorEx masterMotor;

    // "Follower" = motorStanga (Encoder ignored, just follows power)
    private final DcMotorEx followerMotor;

    Follower follower;
    private final Servo hoodServo;
    private final Servo stopper;
    public static double farHoodPose = 0.25;
    public static double closeHoodPose = 0.28;
    public static double veryCloseHoodPose = 0.55;
    public static double middle_Y = 60;
    public static double targetvelocity_compensate = 0;
    public static double stopperClose = 0.37;
    public static double stopperOpen = 0.7;



    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.
    public static double F = 0.0003;
    public static double P = 0.01;
    public static double D = 0;
    public static double I = 0;

    public boolean Manual_shooting = false;
    public static double minPowerDiff = 0.0001;
    public static double lastPower = 0.0;

    public static boolean useLimelight = true;
    public static double add_comp = 0;

    // --- SHOOT-ON-THE-FLY CONSTANTS ---
    // GoBILDA 5000 series 6000RPM, 28 ticks/rev, 72mm flywheel, 0.45 transfer efficiency
    // K_LAUNCHER = 0.45 × π × 0.072m × 39.37in/m / 28 ticks/rev ≈ 0.1431 (in/s per tick/s) transforma getvelocity in viteza mingi de iesire totala
    public static double K_LAUNCHER = 0.1431;
    public static double currentHoodAngleDeg = 47.0;  // updated every loop, read by Turret
    public static double baseTargetVelocity  = 0.0;   // uncompensated velocity, read by Turret

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
        if(this.currentLauncherState != currentLauncherState){
            launcherController.reset();
        }

        this.currentLauncherState = currentLauncherState;
    }

    public Launcher(HardwareMap hwMap, Follower flwr) {
        // 1. Hardware Mapping - HERE is where we define the Master
        masterMotor = hwMap.get(DcMotorEx.class, "motorDreapta"); // MUST have encoder cable
        followerMotor = hwMap.get(DcMotorEx.class, "motorStanga");// Encoder optional/ignored
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        stopper = hwMap.get(Servo.class, "stopper");
        follower = flwr;



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
        // If the robot shoots backward, remove this REVERSE or move it to followerMotor.
        masterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherController = new PIDFController(P, I, D, F);

        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
    }


    void updateLauncherState(){


        switch (currentLauncherState){
            case IDLE:
                if(targetVelocity != 0){
                    //setTargetVelocity(0);
                    setManualVelocity(0);
                }

                break;

            case SHOOTING:
                double currentVel = getVelocity();

                // 2. CALCULATE Error
                double error = requiredSpeed - currentVel;

                launcherController.setSetPoint(requiredSpeed);

                // 3. CALCULATE Power (PF Controller)
                // Feedforward (F): Base power to maintain target
                // Proportional (P): Correction power based on error
                double power = launcherController.calculate(currentVel);


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
                /*if(getVelocity() > getTargetVelocity() - 70){
                    setStopperPose(stopperOpen);
                } else {
                    setStopperPose(stopperClose);
                }
                break;*/
        }
    }



    public void init(){
        setCurrentLauncherState(LauncherState.IDLE);
        //setCurrentStopperState(StopperState.AUTO);
        setHoodPose(farHoodPose);
        setStopperPose(stopperClose);
    }

    /**
     * Sets the target velocity for the flywheel in Ticks Per Second.
     */
    public void setManualVelocity(double velocity) {
        masterMotor.setPower(velocity);
        followerMotor.setPower(velocity);
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }
//    public static void setTargetVelocity(double targetVelocity) {
//        Launcher.targetVelocity = targetVelocity;
//    }
    /**
     * Stops the flywheel.
     */
    public void stop() {
        setCurrentLauncherState(LauncherState.IDLE);
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

    public boolean isVelocityReached() {
        return getVelocity() > requiredSpeed - 50;
    }

    public boolean isStopperOpen(){
        return stopper.getPosition() == stopperOpen;
    }

    /**
     * The heartbeat of the subsystem. This runs constantly to update motor power.
     */
    @Override
    public void periodic() {
        updateLauncherState();

        if (follower.getPose().getY() < middle_Y) {
            setHoodPose(farHoodPose);
            targetvelocity_compensate = -10 + add_comp;
            currentHoodAngleDeg = 47.0;
        } else {
            if(getDistance() <= 58 )
            {
                setHoodPose(veryCloseHoodPose);
                targetvelocity_compensate = 50;//cand e foarte aproape da ft incet
                currentHoodAngleDeg = 31.0;
            } else{
                setHoodPose(closeHoodPose);
                targetvelocity_compensate = 0;
                currentHoodAngleDeg = 38.7;
            }
        }

        // Base (stationary) velocity — also read by Turret for SOF angle+speed compensation
        baseTargetVelocity = Math.pow(getDistance(), 0.4706919) * 189.0741 + targetvelocity_compensate;
        targetVelocity = baseTargetVelocity;

    }
}
