package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

/**
 * Activates shoot-on-the-fly mode: the turret continuously compensates for robot motion
 * using vector subtraction (V_shot = V_ideal - V_robot), and the flywheel speed is
 * adjusted for the radial component of robot velocity.
 *
 * Unlike ShootCommand, this does NOT wait for the turret to reach a fixed setpoint —
 * the turret tracks a moving compensation target continuously while the robot drives.
 */

public class ShootOnFlyCommand extends SequentialCommandGroup {
    public ShootOnFlyCommand(Launcher launcher, Turret turret, Intake intake) {
        addCommands(
                new ParallelCommandGroup(
                        new LauncherStateCommand(launcher, Launcher.LauncherState.SHOOTING),
                        new TurretStateCommand(turret, Turret.TurretState.SHOOT_ON_THE_FLY)
                ),
                // Only wait for flywheel — turret continuously tracks while robot is moving
                new WaitUntilCommand(launcher::isVelocityReached),
                new StopperPoseCommand(launcher, Launcher.stopperOpen),
                new WaitCommand(200),
                new IntakeStateCommand(intake, Intake.IntakeState.REVERSE)
        );
    }
}
