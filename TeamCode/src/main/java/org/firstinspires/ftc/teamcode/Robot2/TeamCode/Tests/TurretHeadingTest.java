package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;

@Config
@TeleOp(name = "Turret Heading Test")
public class TurretHeadingTest extends CommandOpMode {
    DcMotorEx motorTureta;
    Follower follower;
    PIDFController controller;

    double targetAngle = 0;
    public static double kp = 0;
    public static double kd = 0;
    double gearRatio = 3.7;
    double TicksPerRev = 103.8;

    double getTurretHeading(){

        return (motorTureta.getCurrentPosition() / (TicksPerRev * gearRatio)) * 2 * Math.PI ;
    }

    double norm(double angleRadians) {
        while (angleRadians >= Math.PI) {
            angleRadians -= 2.0 * Math.PI;
        }
        while (angleRadians < -Math.PI) {
            angleRadians += 2.0 * Math.PI;
        }
        return angleRadians;
    }

    void update(Pose robotPose, double targetX, double targetY){
        // compute field angle to target
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        double fieldAngle = Math.atan2(dx, dy);


        controller.setP(kp);
        controller.setD(kd);

        // convert to turret-relative angle
        targetAngle = norm(fieldAngle - robotPose.getHeading());


        // Run simple P controller
        double error = norm(targetAngle - getTurretHeading());
        double power = controller.calculate(error, targetAngle);

        motorTureta.setPower(power);
    }

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 7.5, Math.toRadians(90)));

        motorTureta = hardwareMap.get(DcMotorEx.class, "motorTureta");
        motorTureta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTureta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDFController(kp, 0, 0, 0);
        super.reset();

        follower.update();

        super.run();
    }

    public void run(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Turret Heading", getTurretHeading());
        telemetry.addData("Turret Error", norm(targetAngle - getTurretHeading()));
        telemetry.addData("Robot pose", follower.getPose());
        telemetry.addData("motor position", motorTureta.getCurrentPosition());
        telemetry.addData("P", controller.getP());
        telemetry.addData("power", motorTureta.getPower());

        follower.update();

        update(follower.getPose(), 10, 137);

        telemetry.update();
        super.run();
    }
}
