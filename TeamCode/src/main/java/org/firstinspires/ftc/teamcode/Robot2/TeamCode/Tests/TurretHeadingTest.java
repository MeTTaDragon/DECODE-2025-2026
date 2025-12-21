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

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Robot2.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

@Config
@TeleOp(name = "Turret Heading Test")
public class TurretHeadingTest extends CommandOpMode {
    Follower follower;
    Turret turret;



    @Override
    public void initialize(){
        alliance = Alliance.RED;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 7.5, Math.toRadians(90)));

        turret = new Turret(hardwareMap, follower, telemetry);

        super.reset();

        follower.update();

        register(turret);

        turret.setTurretState(Turret.TurretState.IDLE);

        super.run();
    }

    public void run(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower.update();

        turret.setTurretState(Turret.TurretState.FULL_PINPOINT);

        telemetry.update();
        super.run();
    }
}
