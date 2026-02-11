package org.firstinspires.ftc.teamcode.Robot2.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot2.TeamCode.Subsystems.Turret;

public class TurretStateCommand extends InstantCommand {
    private Turret.TurretState state;
    private Turret turret;

    public TurretStateCommand(Turret turretSubsystem, Turret.TurretState state) {
        this.turret = turretSubsystem;
        this.state = state;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turret.setTurretState(state);
    }

}
