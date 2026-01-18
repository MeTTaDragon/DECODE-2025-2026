package org.firstinspires.ftc.teamcode.Robot1.TeamCode.OpMode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Globals;
import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.Robot1.pedroPathing.Constants;

@TeleOp(name = "TeleOp Main robot 1", group = "TeleOpStructures")
public class TeleOpNoPivot extends CommandOpMode {
    private Timer timer;

    GamepadEx chassis;
    GamepadEx second;
    Tun tun;
    Follower follower;

    public static Tun.tunState testState = Tun.tunState.IDLE;


    @Override
    public void initialize() {
        timer = new Timer();

        chassis = new GamepadEx(gamepad1);
        second = new GamepadEx(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Globals.lastAutoPose);

        super.reset();

        tun = new Tun(hardwareMap);


        register(tun);
        tun.init();


        chassis.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> tun.setTunState(tun.getCurrentTunState().equals(Tun.tunState.IDLE) ? Tun.tunState.FORWARD : Tun.tunState.IDLE))
        );

        chassis.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> tun.setBackGatePos(tun.getGateBackPos() == 0 ? tun.gateCloseBack : 0))
        );

        chassis.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> follower.setPose(new Pose(0, 0, Math.toRadians(90))))
        );

        chassis.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> tun.setTunPower(tun.getTunPower() + 0.025))
        );
        chassis.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> tun.setTunPower(tun.getTunPower() - 0.025))
        );
        second.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> tun.setTunPower(tun.getTunPower() + 0.025))
        );
        second.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> tun.setTunPower(tun.getTunPower() - 0.025))
        );

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.startTeleopDrive(true);
        super.run();
    }

    @Override
    public void run() {

        telemetry.setMsTransmissionInterval(250);

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();



        telemetry.addData("Current state", tun.getCurrentTunState());
        telemetry.addData("tun current power", tun.getTunPower());
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();

        super.run();
    }
}
