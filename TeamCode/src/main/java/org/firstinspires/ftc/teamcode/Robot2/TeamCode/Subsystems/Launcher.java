package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Launcher extends SubsystemBase {
    // "Master" = motorDreapta (The one with the Encoder Cable plugged in)
    private final DcMotorEx masterMotor;

    // "Follower" = motorStanga (Encoder ignored, just follows power)
    private final DcMotorEx followerMotor;

    Follower follower;
    Telemetry telemetry;
    private final Servo hoodServo;
    public double farHoodPose = 0.5;
    public double closeHoodPose = 0.1;

    public double middle_Y = 72;


    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.
    public static double F = 0.00036239;
    public static double P = 0.01;
    //vel far zone: 1940
    //vel close middle: 1200
    //vel next to goal:
    private double targetVelocity = 0.0;


    public Launcher(HardwareMap hwMap, Follower flwr, Telemetry telemetry) {
        // 1. Hardware Mapping - HERE is where we define the Master
        masterMotor = hwMap.get(DcMotorEx.class, "motorDreapta"); // MUST have encoder cable
        followerMotor = hwMap.get(DcMotorEx.class, "motorStanga");// Encoder optional/ignored
        hoodServo = hwMap.get(Servo.class, "hoodServo");
        follower = flwr;
        this.telemetry = telemetry;

        // 2. Reset the Master Encoder so we start at 0
        masterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        followerMotor.setDirection(DcMotorSimple.Direction.FORWARD);


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


    /**
     * The heartbeat of the subsystem. This runs constantly to update motor power.
     */
    @Override
    public void periodic() {
        // If target is 0, safety cut power
        if (targetVelocity == 0) {
            masterMotor.setPower(0);
            followerMotor.setPower(0);
            return;
        }
        if (follower.getPose().getY() < middle_Y) {
            setHoodPose(farHoodPose);
        } else {
            setHoodPose(closeHoodPose);
        }
        // 1. READ strictly from motorDreapta (Master)
        double currentVel = getVelocity();

        // 2. CALCULATE Error
        double error = targetVelocity - currentVel;

        // 3. CALCULATE Power (PF Controller)
        // Feedforward (F): Base power to maintain target
        // Proportional (P): Correction power based on error
        double power = (targetVelocity * F) + (error * P);

        // 4. CLAMP power to safe range (-1.0 to 1.0)
        power = Math.max(-1.0, Math.min(1.0, power));

        // 5. APPLY the SAME calculated power to BOTH motors
        // This ensures they stay synced, driven by motorDreapta's encoder data.
        masterMotor.setPower(power);
        followerMotor.setPower(power);
    }
}