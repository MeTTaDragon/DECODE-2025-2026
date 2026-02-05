package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class Intake extends SubsystemBase {

    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.
    public static double F = 0.0004; // Adjust based on motor RPM and Tick Count
    public static double P = 0.0001;
    public static double D = 0;
    public static double I = 0;

    // Target Velocity in Ticks Per Second (TPS)
    // Example: GoBILDA 5202 312RPM is approx 2700 ticks/sec max
    public static double TARGET_VELOCITY_FORWARD = 2000;
    public static double TARGET_VELOCITY_REVERSE = -2300;

    private double targetVelocity = 0.0;

    private DcMotorEx intakeMotor;
    private PIDFController intakeController;

    /**
     * Defines the possible operational states for the Intake subsystem.
     */
    public enum IntakeState {
        FORWARD,
        REVERSE,
        IDLE
    }

    private IntakeState currentIntakeState = IntakeState.IDLE;

    /**
     * Constructs a new Intake subsystem.
     *
     * @param hwMap The hardware map from the OpMode.
     */
    public Intake(HardwareMap hwMap) {
        // 1. Hardware Mapping
        this.intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        // 2. Set to RUN_WITHOUT_ENCODER
        // CRITICAL: Tells the internal REV hub to ignore built-in PID so we can use our own.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 3. Float behavior (optional, allows free spin when 0 power) or BRAKE
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // 4. Initialize the custom Controller
        intakeController = new PIDFController(P, I, D, F);
    }

    /**
     * The core logic loop, structured exactly like the Launcher.
     */
    void updateIntakeState() {
        intakeController.setPIDF(P,I,D,F);
        // 1. Get current velocity
        double currentVel = getCurrentVelocity();

        // 2. Update Controller Setpoint based on state
        switch (currentIntakeState) {
            case IDLE:
                targetVelocity = 0;
                break;
            case FORWARD:
                targetVelocity = TARGET_VELOCITY_FORWARD;
                break;
            case REVERSE:
                targetVelocity = TARGET_VELOCITY_REVERSE;
                break;
        }
        // 3. PIDF Calculation
        // Update controller target
        intakeController.setSetPoint(targetVelocity);

        // Calculate Power
        double power = intakeController.calculate(currentVel);

        // Apply Power
        intakeMotor.setPower(power);

    }

    public void setIntakeState(IntakeState state) {
        if (this.currentIntakeState != state) {
            // Reset controller if we are switching states (optional but good practice)
            intakeController.reset();
        }
        currentIntakeState = state;
    }

    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    public double getCurrentVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    @Override
    public void periodic() {
        // Run the PID Loop
        updateIntakeState();
    }
}