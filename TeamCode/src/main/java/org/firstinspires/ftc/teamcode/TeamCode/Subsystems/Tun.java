package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Tun extends SubsystemBase {
    /**
     * Defines the possible operational states for the Tun subsystem.
     */
    public enum tunState {
        /** The motors and conveyor belt run forward to collect items. */
        FORWARD,
        FORWARD_WITH_PIVOT,
        /** The motors and conveyor belt run in reverse to eject items. */
        REVERSE,
        REVERSE_WITH_PIVOT,
        /** All motors in the subsystem are stopped. */
        IDLE,
        IDLE_WITH_PIVOT
    };


    private static tunState currentTunState;

    public static DcMotorEx motorStanga;
    public static DcMotorEx motorDreapta;
    private CRServo servoBanda;

    public static double TUN_POWER = 0.9;
    public static double BAND_POWER = 0.9;
    public double CURRENT_BAND_POWER = 0.0;

    static double currentSpeedDreapta;

    static double currentSpeedStanga;
    PivotTun pivotTun;


    /**
     * Constructs a new Tun subsystem.
     * This constructor initializes the motors and servos by mapping them to the hardware
     * configuration defined on the Robot Controller.
     *
     * @param hwMap The hardware map from the OpMode, used to access physical devices.
     */
    public Tun(HardwareMap hwMap) {
        this.motorDreapta = hwMap.get(DcMotorEx.class, "motorDreapta");
        this.motorStanga = hwMap.get(DcMotorEx.class, "motorStanga");
        this.servoBanda = hwMap.get(CRServo.class, "servoBanda");
    }


    /**
     * Initializes the Tun subsystem by setting its default state to IDLE.
     */
    public void init()
    {
        setTunState(tunState.IDLE);
    }

    /**
     * Sets the power level for the main motors.
     *
     * @param power The power level to apply, from -1.0 to 1.0.
     */
    public void setTunPower(double power) {
        TUN_POWER = power;
    }

    /**
     * Sets the power level for the conveyor band servo.
     *
     * @param power The power level to apply, from -1.0 to 1.0.
     */
    public void setBandPower(double power) {
        BAND_POWER = power;
    }

    /**
     * Sets the operational state of the Tun subsystem and applies power to the motors accordingly.
     *
     * @param state The desired state (FORWARD, REVERSE, or IDLE).
     */
    public void setTunState(tunState state)
    {
        currentTunState = state;
        switch(currentTunState)
        {
            case FORWARD:
                motorDreapta.setPower(-TUN_POWER);
                motorStanga.setPower(TUN_POWER);

                break;
            case FORWARD_WITH_PIVOT:
                motorDreapta.setPower(-TUN_POWER);
                motorStanga.setPower(TUN_POWER);
                pivotTun.setPivotPosition(1800);


                break;
            case REVERSE:
                motorDreapta.setPower(TUN_POWER);
                motorStanga.setPower(-TUN_POWER);

                break;
            case REVERSE_WITH_PIVOT:
                motorDreapta.setPower(TUN_POWER);
                motorStanga.setPower(-TUN_POWER);
                pivotTun.setPivotPosition(-1800);

                break;
            case IDLE:
                motorDreapta.setPower(0);
                motorStanga.setPower(0);
                break;
            case IDLE_WITH_PIVOT:
                motorDreapta.setPower(0);
                motorStanga.setPower(0);
                pivotTun.setPivotPosition(0);

                break;
        }
    }


    public static double getTunPower()
    {
        return TUN_POWER;
    }
    public double getBandPower()
    {
        return CURRENT_BAND_POWER;
    }
    public static tunState getCurrentTunState()
    {
        return currentTunState;
    }

    public static double getCurrentSpeedDreapta() {return currentSpeedDreapta;}

    public static double getCurrentSpeedStanga() {return currentSpeedStanga;}

    public void periodic() {
        currentSpeedDreapta = motorDreapta.getVelocity();
        currentSpeedStanga = motorStanga.getVelocity();

        if(Math.abs(getCurrentSpeedStanga()) > TUN_POWER * 1000 && getCurrentTunState() == tunState.FORWARD)  {
            servoBanda.setPower(BAND_POWER);
            CURRENT_BAND_POWER = servoBanda.getPower();
        }
        else if(Math.abs(getCurrentSpeedStanga()) > TUN_POWER * 1000 && getCurrentTunState() == tunState.REVERSE) {
            servoBanda.setPower(-BAND_POWER);
            CURRENT_BAND_POWER = servoBanda.getPower();
        }
        else {
            servoBanda.setPower(0);
            CURRENT_BAND_POWER = servoBanda.getPower();
        }

    }

}
