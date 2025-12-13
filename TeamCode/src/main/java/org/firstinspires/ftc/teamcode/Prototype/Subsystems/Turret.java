package org.firstinspires.ftc.teamcode.Prototype.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;



public class Turret extends SubsystemBase {
    private final DcMotorEx launcherMotor1;
    private final DcMotorEx launcherMotor2;
    private final DcMotorEx turretMotor;
    private final Servo hoodServo;
    private final VoltageSensor voltage;


    public Turret(HardwareMap hwMap) {
        launcherMotor1 = hwMap.get(DcMotorEx.class, "motorStanga");
        launcherMotor2 = hwMap.get(DcMotorEx.class, "motorDreapta");
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        hoodServo = hwMap.get(Servo.class, "hoodServo");

        voltage = hwMap.voltageSensor.iterator().next();

        launcherMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcherMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        launcherMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcherMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        launcherMotor1.setPower(power);
        launcherMotor2.setPower(power);
    }

    public void stop() {
        launcherMotor1.setPower(0);
        launcherMotor2.setPower(0);
    }

    @Override
    public void periodic() {
    }


}