package org.firstinspires.ftc.teamcode.Robot1.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
        IDLE,
    };


    private static tunState currentTunState;

    public static DcMotorEx motorStanga;
    public static DcMotorEx motorDreapta;
    private CRServo servoBanda;
    private Servo gateBack;

    public static double TUN_POWER = 0.84;
    public static double BAND_POWER =-1;

    public static double gateCloseBack = 0.02;

    static double currentSpeedDreapta;

    static double currentSpeedStanga;


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
        this.gateBack = hwMap.get(Servo.class, "gateBack");


        gateBack.setDirection(Servo.Direction.REVERSE );

    }


    /**
     * Initializes the Tun subsystem by setting its default state to IDLE.
     */
    public void init()
    {
        setTunState(tunState.IDLE);
        gateBack.setPosition(0);
    }

    /**
     * Sets the power level for the main motors.
     *
     * @param power The power level to apply, from -1.0 to 1.0.
     */
    public void setTunPower(double power) {
        TUN_POWER = power;
        setTunState(currentTunState);
    }

    /**
     * Sets the power level for the conveyor band servo.
     *
     * @param power The power level to apply, from -1.0 to 1.0.
     */
    public void setBandPower(double power) {
        BAND_POWER = power;
    }

    public void setBackGatePos(double pos) {gateBack.setPosition(pos);}


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
                servoBanda.setPower(-BAND_POWER);
                break;

            case REVERSE:
                motorDreapta.setPower(TUN_POWER);
                motorStanga.setPower(-TUN_POWER);
                servoBanda.setPower(-BAND_POWER);

                break;

            case IDLE:
                motorDreapta.setPower(0);
                motorStanga.setPower(0);
                servoBanda.setPower(0);
                break;

        }
    }

    public double getGateBackPos(){return gateBack.getPosition();}
    public static double getTunPower()
    {
        return TUN_POWER;
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


//        if(Math.abs(getCurrentSpeedStanga()) < TUN_POWER * 800 && getCurrentTunState() == tunState.FORWARD)  {
//            gateFront.setPosition(gateCloseFront);
//            gateBack.setPosition(0);
//        }
//        else if(Math.abs(getCurrentSpeedStanga()) < TUN_POWER * 800 && getCurrentTunState() == tunState.REVERSE) {
//            gateFront.setPosition(0);
//            gateBack.setPosition(gateCloseBack);
//        }
//        else {
//            gateFront.setPosition(0);
//            gateBack.setPosition(0);
//        }

    }

}
