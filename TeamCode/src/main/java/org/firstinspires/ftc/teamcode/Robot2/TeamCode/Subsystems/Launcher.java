package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Launcher extends SubsystemBase {
    private final DcMotorEx masterMotor;   // Motor WITH Encoder
    private final DcMotorEx followerMotor; // Motor WITHOUT Encoder (or ignored)
    private final Servo hoodServo;

    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Holds the speed. Start small (e.g., 0.0001)
    // P (Proportional): Fixes errors.
    public static double F = 0.0;
    public static double P = 0.0;

    private double targetVelocity = 0.0;

    public Launcher(HardwareMap hwMap) {
        // Hardware Mapping
        masterMotor = hwMap.get(DcMotorEx.class, "motorDreapta");
        followerMotor = hwMap.get(DcMotorEx.class, "motorStanga");
        hoodServo = hwMap.get(Servo.class, "hoodServo");

        // Reset Master Encoder
        masterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_WITHOUT_ENCODER because we are doing the PID calculation ourselves
        masterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Float behavior allows smoother deceleration
        masterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reverse one motor if they are facing each other
        masterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Sets the target velocity for the flywheel in Ticks Per Second.
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    /**
     * Stops the flywheel.
     */
    public void stop() {
        this.targetVelocity = 0;
        masterMotor.setPower(0);
        followerMotor.setPower(0);
    }

    /**
     * Returns the current velocity from the master motor's encoder.
     */
    public double getVelocity() {
        return masterMotor.getVelocity();
    }

    /**
     * The heartbeat of the subsystem. This runs constantly to update motor power.
     */
    @Override
    public void periodic() {
        // If target is 0, cut power immediately
        if (targetVelocity == 0) {
            masterMotor.setPower(0);
            followerMotor.setPower(0);
            return;
        }

        double currentVel = getVelocity();
        double error = targetVelocity - currentVel;

        // --- Custom PIDF Calculation ---
        // Feedforward (F): Predicting the power needed for the target speed
        // Proportional (P): Nudging the power based on how far off we are
        double power = (targetVelocity * F) + (error * P);

        // Clamp power to ensure it stays within valid range -1 to 1
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply same power to both motors to keep them synced
        masterMotor.setPower(power);
        followerMotor.setPower(power);
    }
}