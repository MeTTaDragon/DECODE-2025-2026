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
 * Activates shoot-on-the-fly mode: the turret continuously compensates for robot motion
 * using vector subtraction (V_shot = V_ideal - V_robot), and the flywheel speed is
 * adjusted for the radial component of robot velocity.
 *
 * Limelight runs in BASKET mode for a 10% fine-trim correction on top of the SOF heading.
 * If limelight doesn't lock (llta == 0), the shot falls back to pure SOF — no stall.
 *
 * Unlike ShootCommand, this does NOT wait for the turret to reach a fixed setpoint —
 * the turret tracks a moving compensation target continuously while the robot drives.
 */
public class ShootOnFlyCommand extends SequentialCommandGroup {
    public ShootOnFlyCommand(Launcher launcher, Turret turret, Intake intake, LimelightSubsystem limelight) {
        addCommands(
                new ParallelCommandGroup(
                        new LauncherStateCommand(launcher, Launcher.LauncherState.SHOOTING),
                        new TurretStateCommand(turret, Turret.TurretState.SHOOT_ON_THE_FLY),
                        new LimelightModeCommand(limelight, LimelightSubsystem.LimelightMode.BASKET)
                ),
                // Only wait for flywheel — turret continuously tracks while robot is moving.
                // Limelight blend is applied automatically in SHOOT_ON_THE_FLY when llta > 0.
                new WaitUntilCommand(launcher::isVelocityReached),
                new StopperPoseCommand(launcher, Launcher.stopperOpen),
                new WaitCommand(200),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE)
        );
    }
}
