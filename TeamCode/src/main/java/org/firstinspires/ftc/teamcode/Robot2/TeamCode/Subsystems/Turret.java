package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends SubsystemBase {

    private final DcMotorEx motorTureta;
    PIDFController turretController;
    Follower follower;
    Telemetry telemetry;

    Pose goalPose;

    public static double P = 0.0, I = 0.0, D = 0.0, F = 0.0;
    double gearRatio = 3.7;
    double TicksPerRev = 103.8;
    double maxAngle;


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
        switch (currentTurretState){
            case IDLE:
                motorTureta.setPower(0);
                break;
            case FULL_LIMELIGHT:
                // Implement Limelight tracking logic here
                break;

            case FULL_PINPOINT:
                Pose2d turretPose = new Pose2d(follower.getPose().getX(), follower.getPose().getY(), getTurretHeading());
                Pose2d goalPose = new Pose2d(this.goalPose.getX(), this.goalPose.getY(), 0);

                double error = errorCalculate(posesToAngle(turretPose, goalPose));

                turretController.setSetPoint(error);

                if(turretController.atSetPoint()){
                    motorTureta.setPower(0);
                    turretController.clearTotalError();
                } else {
                    double power = turretController.calculate(normalizeAngle(getTurretHeading(), false));
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


    /**
     * @param robotPose what the targetPose is being compared to
     * @param targetPose what the robotPose is being compared to
     * @return angle in radians, field-centric, normalized to 0-2pi
     */
     double posesToAngle(Pose2d robotPose, Pose2d targetPose) {
        return normalizeAngle(
                new Vector2d(targetPose).minus(new Vector2d(robotPose)).angle(),
                true
        );
    }

    /**
     * Function to normalize all angles
     *
     * @param angle the angle to be normalized, in degrees or radians
     * @param zeroToMax whether the returned value should be normalized to 0 to 2Pi or -Pi to Pi
     * @return the normalized angle
     */
    double normalizeAngle(double angle, boolean zeroToMax) {
        double max = Math.PI * 2; //radians
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
        double turretHeading = getTurretHeading();
        update();
        turretController.setPIDF(P, I , D, F);

        telemetry.addData("goal pose", goalPose);
        telemetry.addData("turret heading", turretHeading);
        telemetry.addData("turret position", motorTureta.getCurrentPosition());
        telemetry.addData("turret state", currentTurretState);
        telemetry.addData("turret setPoint", turretController.getSetPoint());
        telemetry.addData("turret error", turretController.getPositionError());
        telemetry.addData("turret power", motorTureta.getPower());
        telemetry.addData("robot heading", follower.getPose().getHeading());

    }
}
