package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PController;

@Config
@TeleOp
public class LimelightTrackingTest extends LinearOpMode {

    Limelight3A limelight;
    DcMotorEx motorTureta;
    PController controller;

    public static double tolerance = 0;

    public static double p = 0;
    double tx;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        motorTureta = hardwareMap.get(DcMotorEx.class, "servo");

        limelight.start();
        limelight.pipelineSwitch(0);

        controller = new PController(p);


        waitForStart();
        while (opModeIsActive()) {

            controller.setP(p);
            controller.setTolerance(tolerance);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx();
            } else {
                tx = 0.0;
            }
            double power = controller.calculate(tx, 0);
            motorTureta.setPower(power);

            telemetry.addData("tx", tx);
            telemetry.addData("PID power", power);
            telemetry.addData("motor power", motorTureta.getPower());
            telemetry.addData("MT2", result.getBotpose_MT2());
            telemetry.addData("Position", result.getBotpose()); //cel mai probabil cel bun
            telemetry.addData("Average Distance", result.getBotposeAvgDist());
            telemetry.addData("Average area", result.getBotposeAvgArea());
            telemetry.addData("Pose span", result.getBotposeSpan());
            telemetry.addData("Tag count", result.getBotposeTagCount());
            telemetry.update();
        }
    }
}
