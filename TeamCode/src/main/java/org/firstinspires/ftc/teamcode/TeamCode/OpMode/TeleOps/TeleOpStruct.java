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
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Tun;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp Structure", group = "TeleOpStructures")
public class TeleOpStruct extends CommandOpMode {

    GamepadEx chassis;
    GamepadEx cannon;
    Tun tun;
    PivotTun pivotTun;
    Drivetrain drive;
    Follower follower;

    public static Tun.tunState testState = Tun.tunState.IDLE;
    public static int TARGET_POSITION = 0;

    boolean robotCentric = true;


    @Override
    public void initialize() {
        chassis = new GamepadEx(gamepad1);
        cannon = new GamepadEx(gamepad2);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        super.reset();

        tun = new Tun(hardwareMap);
        pivotTun = new PivotTun(hardwareMap);
        //drive = new Drivetrain(hardwareMap);

        //register(tun, pivotTun, drive);
        register(tun, pivotTun);
        tun.init();
        pivotTun.init();
        //drive.init();

        //drive.setDefaultCommand(new RobotCentricDriveCommand(drive, chassis));


        chassis.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 1700)
        );
        chassis.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, -1700)
        );
        chassis.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    tun.setTunState(Tun.tunState.IDLE);
                })
        );

        chassis.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 0)
        );

        chassis.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> tun.setBackGatePos(tun.getGateBackPos() == 0 ? tun.gateCloseBack : 0))
        );
        chassis.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> tun.setFrontGatePos(tun.getGateFrontPos() == 0 ? tun.gateCloseFront : 0))
        );

        chassis.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> tun.setTunPower(tun.getTunPower() == 0.9 ? 1 : 0.9))
        );

        cannon.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 1700)
        );
        cannon.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, -1700)
        );
        cannon.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    tun.setTunState(Tun.tunState.IDLE);
                })
        );

        cannon.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new setTunDirectionCommand(tun, pivotTun, 0)
        );

        cannon.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> tun.setBackGatePos(tun.getGateBackPos() == 0 ? tun.gateCloseBack : 0))
        );
        cannon.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> tun.setFrontGatePos(tun.getGateFrontPos() == 0 ? tun.gateCloseFront : 0))
        );

        cannon.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> tun.setTunPower(tun.getTunPower() == 0.9 ? 1 : 0.9))
        );

        follower.startTeleopDrive();
        super.run();
    }

    @Override
    public void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

//        telemetry.addData("Motor Stanga Power", tun.motorStanga.getPower());
//        telemetry.addData("Motor Dreapta Power", tun.motorDreapta.getPower());
//        telemetry.addData("Band Power", tun.getBandPower());
//        telemetry.addData("Current state", tun.getCurrentTunState());
//        telemetry.addData("pivot power", pivotTun.getPIVOT_POWER());
//        telemetry.addData("target position", pivotTun.getTARGET_POSITION());
//        telemetry.addData("current position" , pivotTun.getCurrentPosition());
//        telemetry.addData("tolerance", pivotTun.getTolerance());
//        telemetry.addData("tun current power dreapta", tun.getCurrentSpeedDreapta());
//        telemetry.addData("tun current power stanga", tun.getCurrentSpeedStanga());
//        telemetry.addData("robot centric?", robotCentric);
        telemetry.update();

        super.run();
    }
}
