package org.firstinspires.ftc.teamcode.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.Drivetrain;

public class RobotCentricDriveCommand extends CommandBase {
    private final Drivetrain m_drive;
    private final GamepadEx m_gamepad;

    public RobotCentricDriveCommand(Drivetrain drive, GamepadEx gamepad) {
        m_drive = drive;
        m_gamepad = gamepad;
        addRequirements(drive);
    }

    // Runs continuously every loop cycle
    @Override
    public void execute() {
        // Read inputs directly from GamepadEx
        double strafeX = -m_gamepad.getLeftX();
        double forwardY = -m_gamepad.getLeftY();
        double turnZ = -m_gamepad.getRightX();

        // Speed Control (Right Bumper for half speed)
        double driveSpeedMultiplier = m_gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).get() ? 0.4 : 1.0;

        strafeX *= driveSpeedMultiplier;
        forwardY *= driveSpeedMultiplier;
        turnZ *= driveSpeedMultiplier;

        // Call the direct robot-centric drive method.
        m_drive.robotCentricDrive(strafeX, forwardY, turnZ);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setDrivePower(0, 0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false; // Never stop, it's a default drive command
    }
}