package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Robot2.TeamCode.Globals.*;

public class MixedAimCommand extends SequentialCommandGroup {
    public MixedAimCommand(Turret turret, LimelightSubsystem limelight, boolean mixedAim) {
        if(!mixedAim) {
            addCommands(
                    new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET),
                    new TurretStateCommand(turret, Turret.TurretState.FULL_LIMELIGHT)
            );
        }
        else {
            addCommands(
                    new TurretStateCommand(turret, Turret.TurretState.IDLE),
                    new TurretStateCommand(turret, Turret.TurretState.FULL_PINPOINT)
                    //new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET),
                    //new WaitUntilCommand(() -> lltx != 0),
                    //new TurretStateCommand(turret, Turret.TurretState.FULL_LIMELIGHT)
            );
        }
    }
}
