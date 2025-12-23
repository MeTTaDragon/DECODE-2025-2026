package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    // PID Coefficients for Position (Pinpoint)
    public static double P = 0.0, I = 0.0, D = 0.0, F = 0.0;

    // PID Coefficients for Vision (Limelight) - Tune these separately!
    public static double lime_P = 0.02, lime_I = 0.0, lime_D = 0.001;

    double gearRatio = 3.7;
    double TicksPerRev = 103.8;

    // Helper to track if we need to switch PID modes
    private boolean usingLimelightPID = false;

    public enum TurretState {
        IDLE,
        FULL_LIMELIGHT,
        FULL_PINPOINT,
        MIXED
    }
    private static TurretState currentTurretState = TurretState.IDLE;

    public Turret(HardwareMap hwMap, Follower flwr, Telemetry telemetry) {
        motorTureta = hwMap.get(DcMotorEx.class, "motorTureta");



        turretController = new PIDFController(P, I, D, F);

        goalPose = (alliance == Alliance.RED) ? redGoalPose : blueGoalPose;
        follower = flwr;
        this.telemetry = telemetry;
    }

    public void setTurretState(TurretState state) {
        currentTurretState = state;
    }

    void update() {
        double power;

        switch (currentTurretState){
            case IDLE:
                motorTureta.setPower(0);
                break;

            case FULL_LIMELIGHT:
                // 1. Switch controller coefficients to Vision settings
                if (!usingLimelightPID) {
                    turretController.setPIDF(lime_P, lime_I, lime_D, 0);
                    usingLimelightPID = true;
                }

                turretController.setSetPoint(0);

                // Note: Depending on motor direction, you might need to negate this power
                // If tx is positive (target to right), turret needs to turn right.
                power = turretController.calculate(lltx);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                }
                else{
                    motorTureta.setPower(power);
                }


                break;

            case FULL_PINPOINT:
                // 1. Switch controller coefficients back to Position settings
                if (usingLimelightPID) {
                    turretController.setPIDF(P, I, D, F);
                    usingLimelightPID = false;
                }

                Pose2d turretPose = new Pose2d(follower.getPose().getX(), follower.getPose().getY(), getTurretHeading());
                Pose2d targetGoalPose = new Pose2d(this.goalPose.getX(), this.goalPose.getY(), 0);

                double error = errorCalculate(posesToAngle(turretPose, targetGoalPose));

                turretController.setSetPoint(error);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                } else {
                    power = turretController.calculate(normalizeAngle(getTurretHeading(), false));
                    motorTureta.setPower(power);
                }
                break;

            case MIXED:
                // Implement mixed tracking logic here
                break;
        }
    }

    double errorCalculate(double targetAngle){
        double robotAngle = follower. getPose().getHeading();
        double trueHeadding = normalizeAngle(robotAngle + getTurretHeading(), false);
        double error = normalizeAngle(targetAngle - trueHeadding, false);
        return error;
    }

    double posesToAngle(Pose2d robotPose, Pose2d targetPose) {
        return normalizeAngle(
                new Vector2d(targetPose).minus(new Vector2d(robotPose)).angle(),
                true
        );
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

    double getTurretHeading(){
        return - (motorTureta.getCurrentPosition() / (TicksPerRev )) * 2 * Math.PI ;
    }

    @Override
    public void periodic() {
        // Ensure PID values are live-updateable from Dashboard
        if(currentTurretState == TurretState.FULL_LIMELIGHT) {
            turretController.setPIDF(lime_P, lime_I, lime_D, 0);
        } else if (currentTurretState == TurretState.FULL_PINPOINT) {
            turretController.setPIDF(P, I, D, F);
        }

        update();

        double turretHeading = getTurretHeading();
        telemetry.addData("goal pose", goalPose);
        telemetry.addData("turret heading", turretHeading);
        telemetry.addData("turret position", motorTureta.getCurrentPosition());
        telemetry.addData("turret state", currentTurretState);
        telemetry.addData("turret setPoint", turretController.getSetPoint());
        telemetry.addData("turret error", turretController.getPositionError());
        telemetry.addData("robot heading", follower.getPose().getHeading());

        // Added Limelight telemetry
        telemetry.addData("LL tx", lltx);
        telemetry.addData("LL ty", llty);

        telemetry.addData("turret power", motorTureta.getPower());
    }
}