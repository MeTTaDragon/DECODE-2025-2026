package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(Launcher launcher, LimelightSubsystem limelight, Turret turret, Intake intake, boolean mixedAim) {
        addCommands(
                new MixedAimCommand(turret, limelight, mixedAim),
                new WaitUntilCommand(() -> lltx < 1 && launcher.isVelocityReached()),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE)
        );
    }
}
