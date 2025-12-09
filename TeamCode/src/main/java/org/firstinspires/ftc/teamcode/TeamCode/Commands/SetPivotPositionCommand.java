package org.firstinspires.ftc.teamcode.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.TeamCode.Subsystems.PivotTun;

public class SetPivotPositionCommand extends CommandBase {
    private final PivotTun pivotTun;
    private final int targetPosition;

    /**
     * Constructs a new SetPivotPositionCommand.
     * This command sets the target position for the PivotTun subsystem.
     *
     * @param pivotTun       The PivotTun subsystem.
     * @param targetPosition The target encoder position for the pivot motor.
     */
    public SetPivotPositionCommand(PivotTun pivotTun, int targetPosition) {
        this.pivotTun = pivotTun;
        this.targetPosition = targetPosition;

        // Declarăm că această comandă folosește subsistemul PivotTun
        addRequirements(pivotTun);
    }

    @Override
    public void initialize() {
        pivotTun.setPivotPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        // Returnăm true imediat pentru a seta poziția și a termina comanda (fire-and-forget),
        // lăsând PID-ul motorului să își facă treaba în fundal.
        return true;
    }
}