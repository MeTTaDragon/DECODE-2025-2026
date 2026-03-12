package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.AutoCommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.StopperPoseCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands.TurretStateCommand;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

/**
 * Shoots using MIXED turret state: odometry-primary heading (90%) with a small
 * limelight fine-trim on top (10%). No robot velocity compensation.
 *
 * Unlike ShootCommand / MixedAimCommand, this does NOT transition through
 * FULL_PINPOINT → FULL_LIMELIGHT. The turret stays in MIXED the entire time,
 * continuously blending limelight correction into the odometry heading.
 *
 * Waits for BOTH flywheel speed AND turret near-setpoint before opening the stopper,
 * giving the clearest shot possible without adding a hard limelight-lock gate.
 */
public class AutoMixedShootCommand extends SequentialCommandGroup {
    public AutoMixedShootCommand(Launcher launcher, Turret turret, Intake intake) {
        addCommands(
                new WaitUntilCommand(() -> launcher.isVelocityReached() && turret.isNearSetPoint()),
                new StopperPoseCommand(launcher, Launcher.stopperClose),
                new IntakeStateCommand(intake, Intake.IntakeState.SHOOT)
        );
    }
}
