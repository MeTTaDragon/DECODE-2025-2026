package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class PivotTun extends SubsystemBase {
    //TODO: If the motor doesnt move, add "motorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);" to periodic's else

    static DcMotorEx motorPivot;

    //TODO: adauga public daca nu apar in dashboard
    public static double PIVOT_POWER = 0.5;
    public static int TARGET_POSITION = 0;
    public static int tolerance = 0;

    //GETTERS
    public static double getPIVOT_POWER() {
        return PIVOT_POWER;
    }
    public static int getTARGET_POSITION() {
        return TARGET_POSITION;
    }
    public static int getTolerance() {
        return tolerance;
    }
    public static double getCurrentPosition(){
        return motorPivot.getCurrentPosition();
    }

    /**
     * Constructs a new PivotTun subsystem.
     * This constructor initializes the pivot motor, maps it to the hardware configuration,
     * resets its encoder, sets it to brake when idle, and configures its position tolerance.
     *
     * @param hwMap The hardware map from the OpMode, used to access the physical motor.
     */
    public PivotTun(HardwareMap hwMap) {
        this.motorPivot = hwMap.get(DcMotorEx.class, "motorPivot");
        motorPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorPivot.setTargetPositionTolerance(tolerance);
    }


    /**
     * Initializes the pivot motor to its default home position (0).
     * This method sets the target position to 0 and engages the RUN_TO_POSITION mode,
     * preparing the motor to move to and hold its zero position.
     * It is an overloaded version of {@link #init(int)}.
     */
    public void init() {
        setPivotPosition(0);
        motorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Initializes the pivot motor to a specified encoder position.
     * This method sets the motor's target position and ensures it is in RUN_TO_POSITION mode,
     * which is necessary for the periodic() method to control movement.
     *
     * @param position The target encoder tick count for the pivot motor.
     */
    public void init(int position) {
        setPivotPosition(position);
        motorPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets the target position for the pivot motor.
     * This method updates the internal target and sends the command to the motor controller.
     *
     * @param position The desired encoder tick for the motor to move to.
     */
    public void setPivotPosition(int position) {
        TARGET_POSITION = position;
        motorPivot.setTargetPosition(TARGET_POSITION);
    }
    //just a method overload for easier tests
    public void setPivotPosition() {
        motorPivot.setTargetPosition(TARGET_POSITION);
    }

    /**
     * This method should be called repeatedly in a loop to drive the motor to its target.
     * It applies a constant power until the motor reaches its target position, at which point it stops the motor.
     */
    @Override
    public void periodic() {
        if (motorPivot.getCurrentPosition() == motorPivot.getTargetPosition())
            motorPivot.setPower(0);

        else motorPivot.setPower(PIVOT_POWER);
    }
}
