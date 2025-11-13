package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        /** The motors and conveyor belt run in reverse to eject items. */
        REVERSE,
        /** All motors in the subsystem are stopped. */
        IDLE
    };


    private static tunState currentTunState;

    private DcMotor motorStanga;
    private DcMotor motorDreapta;
    private CRServo servoBanda;

    public static double TUN_POWER = 0.5;
    public static double BAND_POWER = 0.5;


    /**
     * Constructs a new Tun subsystem.
     * This constructor initializes the motors and servos by mapping them to the hardware
     * configuration defined on the Robot Controller.
     *
     * @param hwMap The hardware map from the OpMode, used to access physical devices.
     */
    public Tun(HardwareMap hwMap) {
        this.motorDreapta = hwMap.get(DcMotor.class, "motorDreapta");
        this.motorStanga = hwMap.get(DcMotor.class, "motorStanga");
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
                motorDreapta.setPower(TUN_POWER);
                motorStanga.setPower(-TUN_POWER);
                servoBanda.setPower(BAND_POWER);
                break;
            case REVERSE:
                motorDreapta.setPower(-TUN_POWER);
                motorStanga.setPower(TUN_POWER);
                servoBanda.setPower(-BAND_POWER);
                break;
            case IDLE:
                motorDreapta.setPower(0);
                motorStanga.setPower(0);
                servoBanda.setPower(0);
                break;
        }
    }


    public static double getTunPower()
    {
        return TUN_POWER;
    }
    public static double getBandPower()
    {
        return BAND_POWER;
    }
    public static tunState getCurrentTunState()
    {
        return currentTunState;
    }
}
