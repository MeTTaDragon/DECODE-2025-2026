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


    public static double P = 0.006, I = 0.005, D = 0.00025, F = 0.0;

    double gearRatio = 3.7; //pune asta cand o sa stiu exact gear ratio-ul de la tureta
    double TicksPerRev = 103.8;


    public enum TurretState {
        IDLE,
        FULL_LIMELIGHT,
        FULL_PINPOINT,
        MIXED
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

    void update() {
        double power;

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
                    motorTureta.setPower(power);
                }
                break;

            case FULL_PINPOINT:
                turretPose = new Pose2d(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());

                double targetAngle = posesToAngle(turretPose, targetGoalPose);

                double error = errorCalculate(targetAngle);

                turretController.setSetPoint(error);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                } else {
                    power = turretController.calculate();
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
        double trueHeading = normalizeAngle(robotAngle + getTurretHeading(), false);

        double error = normalizeAngle(targetAngle - trueHeading, false);

        return error;
    }

    double posesToAngle(Pose2d robotPose, Pose2d targetPose) {

        return new Vector2d(targetPose).minus(new Vector2d(robotPose)).angle();

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
        return (motorTureta.getCurrentPosition() / (TicksPerRev )) * 2 * Math.PI ;
    }

    @Override
    public void periodic() {

        turretController.setPIDF(P, I, D, F);


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