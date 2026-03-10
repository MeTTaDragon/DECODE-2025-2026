package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
public class Intake extends SubsystemBase {

    // --- TUNING VARIABLES (Edit in FTC Dashboard) ---
    // F (Feedforward): Base power to hold speed. Start small (0.0001 - 0.0005)
    // P (Proportional): "Snap" power to fix errors.

    private double targetVelocity = 0.0;

    private DcMotorEx intakeMotor;

    /**
     * Defines the possible operational states for the Intake subsystem.
     */
    public enum IntakeState {
        INTAKE,
        SHOOT,
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

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Float behavior (optional, allows free spin when 0 power) or BRAKE
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    /**
     * The core logic loop, structured exactly like the Launcher.
     */
    void updateIntakeState() {

        // 2. Update Controller Setpoint based on state
        switch (currentIntakeState) {
            case IDLE:
                targetVelocity = 0;
                break;
            case SHOOT:
                targetVelocity = 0.8;
                break;
            case INTAKE:
                targetVelocity = 1;
                break;
            case REVERSE:
                targetVelocity = -1;
                break;
        }

        // Apply Power
        intakeMotor.setPower(targetVelocity);

    }

    public void setIntakeState(IntakeState state) {

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