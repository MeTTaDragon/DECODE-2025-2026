package org.firstinspires.ftc.teamcode.TeamCode.OpMode.TeleOpTests;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.TeamCode.Commands.RobotCentricDriveCommand;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Drivetrain;

@TeleOp(name = "Test DriveTrain", group = "TeleOp Tests")
public class TestDriveTrain extends CommandOpMode {

    private Drivetrain drive;
    private GamepadEx gamepad;

    public void initialize() {
        drive = new Drivetrain(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        
        super.reset();
        register(drive);
        drive.init();

        drive.setDefaultCommand(new RobotCentricDriveCommand(drive, gamepad));



    }



    @Override
    public void run(){
        super.run(); // Executes the command scheduler and calls subsystem periodic()

        // Telemetry updates for driver feedback
        telemetry.addData("Drive Mode", "ROBOT CENTRIC");
        telemetry.update();
    }
}
