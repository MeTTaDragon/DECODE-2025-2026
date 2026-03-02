package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.LimelightSubsystem;
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
public class MixedShootCommand extends SequentialCommandGroup {
    public MixedShootCommand(Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight) {
        addCommands(
                new ParallelCommandGroup(
                        new LauncherStateCommand(launcher, Launcher.LauncherState.SHOOTING),
                        new TurretStateCommand(turret, Turret.TurretState.MIXED),
                        new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET)
                ),
                new WaitUntilCommand(() -> launcher.isVelocityReached() && turret.isNearSetPoint()),
                new StopperPoseCommand(launcher, Launcher.stopperOpen),
                new IntakeStateCommand(intake, Intake.IntakeState.SHOOT)
        );
    }
}
