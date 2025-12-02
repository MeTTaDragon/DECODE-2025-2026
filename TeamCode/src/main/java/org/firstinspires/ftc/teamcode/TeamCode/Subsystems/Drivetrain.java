package org.firstinspires.ftc.teamcode.TeamCode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain extends SubsystemBase {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    
    private double currentHeading = 0.0;




    public Drivetrain(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");




    }

    public void init() {
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void robotCentricDrive(double strafeX, double forwardY, double turnZ) {

        // Denominator used for normalization to keep power within [-1, 1]
        double denominator = Math.max(Math.abs(forwardY) + Math.abs(strafeX) + Math.abs(turnZ), 1.0);

        double frontLeftPower = (forwardY + strafeX + turnZ) / denominator;
        double backLeftPower = (forwardY - strafeX + turnZ) / denominator;
        double frontRightPower = (forwardY - strafeX - turnZ) / denominator;
        double backRightPower = (forwardY + strafeX - turnZ) / denominator;

        setDrivePower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeft.setPower(Range.clip(fl, -0.5, 0.5));
        backLeft.setPower(Range.clip(bl, -0.5, 0.5));
        frontRight.setPower(Range.clip(fr, -0.5, 0.5));
        backRight.setPower(Range.clip(br, -0.5, 0.5));
    }




}
