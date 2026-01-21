package org.firstinspires.ftc.teamcode.Robot2.TeamCode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.Robot1.pedroPathing.Constants;

public class TeleOpMain extends CommandOpMode {

    GamepadEx controller;

    Follower follower;

    @Override
    public void initialize() {
        controller = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Globals.lastAutoPose);

        super.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.startTeleopDrive(true);
        super.run();

    }


    public void run() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
    }
}
