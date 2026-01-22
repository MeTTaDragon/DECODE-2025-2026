package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import android.opengl.EGLObjectHandle;

@Config
@TeleOp(name = "Turret Heading Test")
public class TurretHeadingTest extends CommandOpMode {
    Follower follower;
    Turret turret;
    LimelightSubsystem limelight;
    GamepadEx gamepad;

    DcMotor motorIntake;
    DcMotor motorStanga;
    DcMotor motorDreapta;



    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        alliance = Alliance.BLUE;

       follower = Constants.createFollower(hardwareMap);
       follower.setStartingPose(new Pose(72, 7.5, Math.toRadians(90)));

        turret = new Turret(hardwareMap, follower, telemetry);
        gamepad = new GamepadEx(gamepad1);
        motorIntake = hardwareMap.get(DcMotor.class, "intakeMotor");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStanga = hardwareMap.get(DcMotor.class, "motorStanga");
        motorDreapta = hardwareMap.get(DcMotor.class, "motorDreapta");
        motorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        //limelight = new LimelightSubsystem(hardwareMap);

        super.reset();

        follower.update();

        register(turret);
        //register(limelight);

        turret.setTurretState(Turret.TurretState.IDLE);
//        limelight.init();

//        limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.FULL_PINPOINT))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> turret.setTurretState(Turret.TurretState.IDLE))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> motorIntake.setPower(0.7))
        );
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    motorDreapta.setPower(0.6);
                    motorStanga.setPower(0.6);
                })
        );


        super.run();
    }

    public void run(){
        follower.update();

        telemetry.addData("Current state", turret.getCurrentTurretState());
        telemetry.addData("Turret Power", turret.getCurrentPower());
        telemetry.addData("turret heading", Math.toDegrees(turret.getTurretHeading()));
        telemetry.addData("Set Point", Math.toDegrees(turret.getSetPoint()));
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("launcher", motorDreapta.getPower());
        telemetry.update();
        super.run();
    }
}
