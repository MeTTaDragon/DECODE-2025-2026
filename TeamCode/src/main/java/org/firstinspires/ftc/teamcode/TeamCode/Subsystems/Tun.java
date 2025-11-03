package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Tun extends SubsystemBase {
    public enum tunState {
        FORWARD,
        REVERSE,
        IDLE
    };

    public enum pivotState{
        FORWARD, //it corresponds to the tunState direction.
        REVERSE, //if the pivotState = FORWARD, the tunState should also = FORWARD
        IDLE
    }

    private static tunState currentTunState;
    private static pivotState currentPivotState;

    private DcMotor motorStanga;
    private DcMotor motorDreapta;
    private DcMotor motorPivot;

    private CRServo servoBanda;

    public static double TUN_POWER = 0.5;
    public static double BAND_POWER = 0.5;
    public static double PIVOT_POWER = 0.5;

    public Tun(HardwareMap hwMap)
    {
        this.motorDreapta = hwMap.get(DcMotor.class, "motorDreapta");
        this.motorStanga = hwMap.get(DcMotor.class, "motorStanga");
        this.servoBanda = hwMap.get(CRServo.class, "servoBanda");
        //this.motorPivot = hwMap.get(DcMotor.class, "motorPivot");
    }

    public void init()
    {
        currentTunState = tunState.IDLE;
        currentPivotState = pivotState.IDLE;
    }

    public void setTunPower(double power) {
        TUN_POWER = power;
    }
    public void setBandPower(double power) {
        BAND_POWER = power;
    }
    public void setPivotower(double power) {
        PIVOT_POWER = power;
    }

    public void setTunState(tunState state)
    {
        currentTunState = state;
        switch(currentTunState)
        {
            case FORWARD:
                motorDreapta.setPower(TUN_POWER);
                motorStanga.setPower(TUN_POWER);
                servoBanda.setPower(BAND_POWER);
                break;
            case REVERSE:
                motorDreapta.setPower(TUN_POWER);
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
