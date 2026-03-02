package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;


public class ShootCommand extends SequentialCommandGroup {
    /**
     * Creates a new ShootCommand.
     * This command will aim the turret, wait for the launcher to be at speed and the turret to be on target, and then run the intake in reverse to shoot.
     *
     * @param launcher The launcher subsystem.
     * @param limelight The limelight subsystem.
     * @param turret The turret subsystem.
     * @param intake The intake subsystem.
     * @param mixedAim Whether to use mixed aim.
     */
    public ShootCommand(Launcher launcher, LimelightSubsystem limelight, Turret turret, Intake intake, boolean mixedAim) {
        addCommands(
                new MixedAimCommand(turret, limelight, mixedAim),
                new WaitUntilCommand(() -> (!mixedAim || lltx < 2) && launcher.isVelocityReached()),
                new InstantCommand(() -> launcher.setRampPos(1)),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE)
        );
    }
}
