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
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

@Config
@TeleOp(name = "Turret Heading Test")
public class TurretHeadingTest extends CommandOpMode {
    Follower follower;
    Turret turret;
    LimelightSubsystem limelight;



    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        alliance = Alliance.BLUE;

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 7.5, Math.toRadians(90)));

        turret = new Turret(hardwareMap, null, telemetry);
        //limelight = new LimelightSubsystem(hardwareMap);

        super.reset();

        //follower.update();

        register(turret);
        //register(limelight);

        turret.setTurretState(Turret.TurretState.FULL_PINPOINT);
//        limelight.init();
//        limelight.setMode(LimelightSubsystem.LimelightMode.BASKET);

        super.run();
    }

    public void run(){
        //follower.update();



        telemetry.update();
        super.run();
    }
}
