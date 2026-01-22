package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Launcher extends SubsystemBase {
    private final DcMotorEx launcherMotor1;
    private final DcMotorEx launcherMotor2;
    Follower follower;

    private final Servo hoodServo;
    private final VoltageSensor voltage;
    public double farHoodPose = 0.5;
    public double closeHoodPose = 0.1;

    public double middle_X = 72;


    public Launcher(HardwareMap hwMap, Follower flwr, Telemetry telemetry) {
        launcherMotor1 = hwMap.get(DcMotorEx.class, "motorStanga");
        launcherMotor2 = hwMap.get(DcMotorEx.class, "motorDreapta");
        hoodServo = hwMap.get(Servo.class, "hoodServo");

        follower = flwr;

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

    public void setHoodPose(double pos) {
        hoodServo.setPosition(pos);
    }
    @Override
    public void periodic() {
        if (follower.getPose().getX() > middle_X) {
            setHoodPose(farHoodPose);
        } else {
            setHoodPose(closeHoodPose);
        }

    }


}