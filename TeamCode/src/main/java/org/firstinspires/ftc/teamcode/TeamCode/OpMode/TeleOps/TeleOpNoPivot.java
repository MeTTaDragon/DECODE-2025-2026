package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Commands.RobotCentricDriveCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Commands.setTunDirectionCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp Main", group = "TeleOpStructures")
public class TeleOpNoPivot extends CommandOpMode {

    GamepadEx chassis;
    Tun tun;
    Follower follower;

    public static Tun.tunState testState = Tun.tunState.IDLE;


    @Override
    public void initialize() {
        chassis = new GamepadEx(gamepad1);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Globals.lastAutoPose);

        super.reset();

        tun = new Tun(hardwareMap);


        register(tun);
        tun.init();


        chassis.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> tun.setTunState(Tun.tunState.IDLE))
        );
        chassis.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> tun.setTunState(Tun.tunState.REVERSE))
        );
        chassis.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> tun.setTunState(Tun.tunState.FORWARD))
        );

        chassis.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> tun.setBackGatePos(tun.getGateBackPos() == 0 ? tun.gateCloseBack : 0))
        );

        chassis.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> follower.setPose(new Pose(0, 0, Math.toRadians(90))))
        );


        follower.startTeleopDrive(true);
        super.run();
    }

    @Override
    public void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(250);

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();


        telemetry.addData("Current state", tun.getCurrentTunState());
        telemetry.addData("tun current power dreapta", tun.getCurrentSpeedDreapta());
        telemetry.addData("tun current power stanga", tun.getCurrentSpeedStanga());
        telemetry.update();

        super.run();
    }
}
