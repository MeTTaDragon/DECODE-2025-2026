package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry; // Import Telemetry

import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Intake extends SubsystemBase {

    /**
     * Defines the possible operational states for the Intake subsystem.
     */
    public enum IntakeState {
        FORWARD,
        REVERSE,
        IDLE
    }

    private IntakeState currentIntakeState = IntakeState.IDLE;
    private DcMotorEx intakeMotor;


    // We store the telemetry object to use it in periodic()
    private Telemetry telemetry;

    public static double INTAKE_POWER = 1;

    /**
     * Constructs a new Intake subsystem.
     *
     * @param hwMap The hardware map from the OpMode.
     * @param telemetry The telemetry object to display data on the Driver Station.
     */
    public Intake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize the motor
        this.intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        // Optional: motor configuration
        // intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the operational state of the Intake subsystem.
     */
    public void setIntakeState(IntakeState state) {
        currentIntakeState = state;

        switch (currentIntakeState) {
            case FORWARD:
                intakeMotor.setPower(INTAKE_POWER);
                break;
            case REVERSE:
                intakeMotor.setPower(-INTAKE_POWER);
                break;
            case IDLE:
            default:
                intakeMotor.setPower(0);
                break;
        }
    }

    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    public void setIntakePower(double power) {
        INTAKE_POWER = power;
        // Refresh state to apply new power immediately if active
        if (currentIntakeState != IntakeState.IDLE) {
            setIntakeState(currentIntakeState);
        }
    }

    /**
     * This method runs repeatedly while the OpMode is active.
     * We use it here to push Telemetry data.
     */
    @Override
    public void periodic() {
        telemetry.addData("Intake State", currentIntakeState);

        telemetry.addData("Intake Target Power", INTAKE_POWER);

        telemetry.addData("Intake Motor Power", intakeMotor.getPower());

        telemetry.addData("Intake Velocity", intakeMotor.getVelocity());
    }
}