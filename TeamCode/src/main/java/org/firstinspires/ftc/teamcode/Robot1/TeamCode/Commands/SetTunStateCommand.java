package org.firstinspires.ftc.teamcode.Robot1.TeamCode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Robot1.TeamCode.Subsystems.Tun;

public class SetTunStateCommand extends CommandBase {
    private final Tun tun;
    private final Tun.tunState targetState;

    /**
     * Constructs a new SetTunStateCommand.
     * This command sets the running state (power/direction) of the Tun subsystem.
     *
     * @param tun         The Tun subsystem.
     * @param targetState The desired state (FORWARD, REVERSE, IDLE, etc.).
     */
    public SetTunStateCommand(Tun tun, Tun.tunState targetState) {
        this.tun = tun;
        this.targetState = targetState;

        // Declarăm că această comandă folosește subsistemul Tun
        addRequirements(tun);
    }

    @Override
    public void initialize() {
        tun.setTunState(targetState);
    }

    @Override
    public boolean isFinished() {
        // Comanda se termină instant după setarea stării.
        return true;
    }
}