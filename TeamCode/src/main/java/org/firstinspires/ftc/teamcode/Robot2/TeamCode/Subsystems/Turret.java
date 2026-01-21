package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals;

@Config
public class Turret extends SubsystemBase {

    private final DcMotorEx motorTureta;

    PIDFController turretController;
    Follower follower;
    Telemetry telemetry;

    Pose goalPose;
    Pose2d targetGoalPose;
    Pose2d turretPose;

    double robotAngle;
    double power;


    public static double P = 0.35, I = 0, D = 0.0012, F = 0;

    double gearRatio = 5.75;
    double TicksPerRev = 145.1;
    double testPoint;


    public enum TurretState {
        IDLE,
        FULL_LIMELIGHT,
        FULL_PINPOINT,
        MIXED,
        TEST
    }
    private static TurretState currentTurretState = TurretState.IDLE;

    public Turret(HardwareMap hwMap, Follower flwr, Telemetry telemetry) {
        motorTureta = hwMap.get(DcMotorEx.class, "motorTureta");
        motorTureta.setDirection(DcMotorSimple.Direction.REVERSE);

        turretController = new PIDFController(P, I, D, F);

        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
        follower = flwr;
        this.telemetry = telemetry;

        targetGoalPose = new Pose2d(this.goalPose.getX(), this.goalPose.getY(), 0);
    }

    public void setTurretState(TurretState state) {
        currentTurretState = state;
    }
    public TurretState getCurrentTurretState(){ return currentTurretState; }
    public double getPower(){ return motorTureta.getPower();}
    public void setTestPoint(double point){ testPoint = point; }
    void update() {
        switch (currentTurretState){
            case IDLE:
                motorTureta.setPower(0);
                break;

            case FULL_LIMELIGHT:
                turretController.setSetPoint(0);

                power = turretController.calculate(lltx);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                }
                else{
                    motorTureta.setPower(-power);
                }
                break;

            case FULL_PINPOINT:
                turretPose = new Pose2d(follower.getPose().getX(), follower.getPose().getY(), getTurretHeading());

                double targetAngle = posesToAngle(turretPose, targetGoalPose);

                double error = errorCalculate(targetAngle);

                turretController.setSetPoint(targetAngle);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                } else {
                    power = turretController.calculate(getTurretHeading());
                    motorTureta.setPower(power);
                }
                break;

            case MIXED:
                // Implement mixed tracking logic here
                break;

            case TEST:
                turretController.setSetPoint(testPoint);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                } else {
                    double trueHeading = normalizeAngle(getTurretHeading(), true);
                    power = turretController.calculate(trueHeading);
                    motorTureta.setPower(power);
                }
        }
    }

    double errorCalculate(double targetAngle){

        double error = calcError2(targetAngle, getTurretHeading());


        return error;
    }

    double calcError2(double targetangle, double trueheading) {
        if ((targetangle - trueheading < -180) || (targetangle - trueheading > 180)) {
            return (targetangle + trueheading);
        }
        else {
            return (targetangle - trueheading);
        }
    }

    double posesToAngle(Pose2d robotPose, Pose2d targetPose) {

        double deltaY = targetPose.getY() - robotPose.getY();
        double deltaX = targetPose.getX() - robotPose.getX();

        double targetAngle = Math.atan2(deltaY, deltaX);

        return targetAngle;
    }

    double normalizeAngle(double angle, boolean zeroToMax) {
        double max = Math.PI * 2;
        double angle2 = angle % max;
        if (zeroToMax && angle2 < 0) {
            return angle2 + max;
        } else if (!zeroToMax) {
            if (angle2 > max/2) {
                return angle2 - max;
            } else if (angle2 < -max/2) {
                return angle2 + max;
            }
        }
        return angle2;
    }



    public double getTurretHeading(){
        double turretHeading = (motorTureta.getCurrentPosition() / (TicksPerRev * gearRatio)) * 2 * Math.PI ;

        double trueHeading = turretHeading + robotAngle;

        return trueHeading;
    }

    public double getCurrentPower(){
        return power;
    }


    public double getSetPoint(){ return turretController.getSetPoint(); }

    @Override
    public void periodic() {

        turretController.setPIDF(P, I, D, F);

        robotAngle = follower.getPose().getHeading();

        update();
    }
}