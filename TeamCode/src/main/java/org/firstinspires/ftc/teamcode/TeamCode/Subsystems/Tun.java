package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Tun extends SubsystemBase {
    public enum tunState {
        FORWARD,
        REVERSE,
        STOP
    };

    private tunState currentTunState;

    private DcMotor motorStanga;
    private DcMotor motorDreapta;

    private CRServo servoBanda;

    public static double TUN_POWER = 0.5;
    public static double BAND_POWER = 0.5;

    public Tun(HardwareMap hwMap)
    {
        this.motorDreapta = hwMap.get(DcMotor.class, "motorDreapta");
        this.motorStanga = hwMap.get(DcMotor.class, "motorStanga");
        this.servoBanda = hwMap.get(CRServo.class, "servoBanda");
    }

    public void init()
    {
        currentTunState=tunState.STOP;
    }

    public void setTunPower(double power) {
        TUN_POWER=power;
    }

    public void setBandPower(double power) {
        BAND_POWER=power;
    }

    public void setTunState(tunState state)
    {
        currentTunState=state;
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
            case STOP:
                motorDreapta.setPower(0);
                motorStanga.setPower(0);
                servoBanda.setPower(0);
                break;
        }
    }

    public void stop()
    {
        motorDreapta.setPower(0);
        motorStanga.setPower(0);
        servoBanda.setPower(0);
    }

    public double getTunPower()
    {
        return TUN_POWER;
    }

    public double getBandPower()
    {
        return BAND_POWER;
    }

    public tunState getCurrentTunState()
    {
        return currentTunState;
    }


}
